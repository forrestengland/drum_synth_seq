/*
 * xtrack - small ncurses music tracker
 *
 *   - fits in an 80x24 terminal
 *   - JACK stereo audio out, JACK MIDI in (clock / start / stop / continue / SPP)
 *   - 4 tracker channels, 64-row pattern, 8 instrument slots
 *   - cell = note, instrument number, 2-digit hex volume
 *   - instruments: sample import (anything libsndfile reads), tune (semitones),
 *     fine tune (cents), volume
 *
 * Usage:  ./xtrack [sample1.wav ... sample8.wav]
 *
 * Keys (global)
 *   F1 pattern view        F2 instrument view
 *   F3/F4 octave -/+       F9 sync INT <-> MIDI      F10 or Ctrl-C quit
 *   Space play/stop        < > BPM -/+  (internal sync)
 *
 * Pattern view
 *   arrows / PgUp PgDn / Home End   move       Tab / Shift-Tab   next/prev channel
 *   note field: piano keys  (z s x d c v g b h n j m = C..B of current octave,
 *               q 2 w 3 e r 5 t 6 y 7 u i 9 o 0 p = one octave up),  1 = note off
 *   inst field: 1-8        volume fields: 0-9 a-f (hex)
 *   [ ]   previous/next instrument     ( )  edit step -/+
 *   Del   clear field (clears whole cell on note field)
 *   Enter play from cursor
 *
 * Instrument view
 *   Up/Down select slot   Tab select param (tune/fine/vol)   Left/Right adjust
 *   PgUp/PgDn big adjust  L or Enter load sample             Del unload
 *   piano keys audition the selected instrument
 */
#define _GNU_SOURCE
#include <ncurses.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <sndfile.h>
#include <stdatomic.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <locale.h>

#define NCHAN      4
#define NINST      8
#define PAT_LEN    64
#define NOTE_OFF   97
#define BASE_NOTE  49                 /* C-4: sample plays at original pitch */
#define MAX_FRAMES (8 * 1024 * 1024)
#define NVOICE     (NCHAN + 1)        /* extra voice = audition */
#define PREVIEW    NCHAN
#define MIX_GAIN   0.5f
#define PEAK_COLS  76

/* ------------------------------------------------------------------ data */

typedef struct { uint8_t note, inst; int16_t vol; } Cell;   /* vol -1 = none */

typedef struct {
    float   *data;               /* mono float */
    uint32_t len, rate;
    int      tune;               /* semitones  -48..48 */
    int      fine;               /* cents      -99..99 */
    int      vol;                /* 0..255 */
    char     name[24];
} Inst;

typedef struct { int active, inst; double pos, step; float gain; } Voice;

static Cell  pat[PAT_LEN][NCHAN];
static Inst  ins[NINST];
static Voice voices[NVOICE];
static const float pan[NVOICE] = { 0.25f, 0.75f, 0.75f, 0.25f, 0.5f };

/* instrument table is guarded by this; audio thread only try-locks */
static pthread_mutex_t ins_lock = PTHREAD_MUTEX_INITIALIZER;

/* UI -> audio / audio -> UI */
static atomic_int bpm        = 120;
static atomic_int sync_ext   = 0;
static atomic_int playing    = 0;
static atomic_int play_row   = 0;
static atomic_int cmd        = 0;      /* 1 start, 2 stop */
static atomic_int cmd_row    = 0;
static atomic_int preview_req = 0;     /* ((inst+1)<<8) | note */
static atomic_int bpm_est10  = 0;      /* measured MIDI clock tempo * 10 */

/* audio-thread private */
static jack_client_t *client;
static jack_port_t   *out_l, *out_r, *midi_in;
static jack_nframes_t srate = 48000;
static int      next_row, running, clk;
static double   acc, spr, clk_avg;
static uint64_t abs_frames, last_clk_abs;
static int      last_inst[NCHAN] = { -1, -1, -1, -1 };

/* ----------------------------------------------------------------- audio */

static void kill_voices(void)
{
    for (int v = 0; v < NCHAN; v++) voices[v].active = 0;
}

static void voice_trigger(int v, int inst, int note, float vol)
{
    Inst *in = &ins[inst];
    if (!in->data || !in->len) return;
    double semis = (note - BASE_NOTE) + in->tune + in->fine / 100.0;
    Voice *vc = &voices[v];
    vc->inst = inst;
    vc->pos  = 0;
    vc->step = ((double)in->rate / (double)srate) * pow(2.0, semis / 12.0);
    vc->gain = vol * (in->vol / 255.0f);
    vc->active = 1;
}

static void step_row(void)
{
    int r = next_row;
    next_row = (r + 1) % PAT_LEN;
    atomic_store(&play_row, r);

    for (int c = 0; c < NCHAN; c++) {
        Cell *ce = &pat[r][c];
        if (ce->note == NOTE_OFF) {
            voices[c].active = 0;
        } else if (ce->note >= 1 && ce->note <= 96) {
            int i = ce->inst ? ce->inst - 1 : last_inst[c];
            if (i < 0 || i >= NINST) continue;
            last_inst[c] = i;
            voice_trigger(c, i, ce->note, ce->vol < 0 ? 1.0f : ce->vol / 255.0f);
        } else if (ce->vol >= 0 && voices[c].active) {
            voices[c].gain = (ce->vol / 255.0f) * (ins[voices[c].inst].vol / 255.0f);
        }
    }
}

static void render(float *L, float *R, jack_nframes_t from, jack_nframes_t to)
{
    for (int v = 0; v < NVOICE; v++) {
        Voice *vc = &voices[v];
        if (!vc->active) continue;
        Inst *in = &ins[vc->inst];
        if (!in->data) { vc->active = 0; continue; }
        float pr = pan[v], pl = 1.0f - pr;
        float g  = vc->gain * MIX_GAIN;
        for (jack_nframes_t i = from; i < to; i++) {
            uint32_t idx = (uint32_t)vc->pos;
            if (idx >= in->len) { vc->active = 0; break; }
            float fr = (float)(vc->pos - idx);
            float s0 = in->data[idx];
            float s1 = (idx + 1 < in->len) ? in->data[idx + 1] : 0.0f;
            float s  = (s0 + (s1 - s0) * fr) * g;
            L[i] += s * pl;
            R[i] += s * pr;
            vc->pos += vc->step;
        }
    }
}

static void handle_midi(const jack_midi_event_t *ev, int ext)
{
    if (ev->size < 1) return;
    unsigned char st = ev->buffer[0];
    switch (st) {
    case 0xF8: {                                   /* clock, 24 ppqn */
        uint64_t t = abs_frames + ev->time;
        if (last_clk_abs) {
            double d = (double)(t - last_clk_abs);
            if (d > srate / 2.0 || d <= 0) clk_avg = 0;
            else {
                clk_avg = clk_avg == 0 ? d : clk_avg * 0.95 + d * 0.05;
                atomic_store(&bpm_est10, (int)(600.0 * srate / (24.0 * clk_avg)));
            }
        }
        last_clk_abs = t;
        if (ext && running) {
            if (clk % 6 == 0) step_row();          /* 6 clocks = one 16th */
            clk++;
        }
        break;
    }
    case 0xFA: if (ext) { next_row = 0; clk = 0; running = 1; } break;   /* start */
    case 0xFB: if (ext) running = 1; break;                              /* continue */
    case 0xFC: if (ext) { running = 0; kill_voices(); } break;           /* stop */
    case 0xF2:                                                           /* SPP */
        if (ext && ev->size >= 3) {
            int spp = (ev->buffer[1] & 0x7f) | ((ev->buffer[2] & 0x7f) << 7);
            next_row = spp % PAT_LEN;
            clk = 0;
        }
        break;
    }
}

static int process(jack_nframes_t n, void *arg)
{
    (void)arg;
    float *L = jack_port_get_buffer(out_l, n);
    float *R = jack_port_get_buffer(out_r, n);
    memset(L, 0, n * sizeof(float));
    memset(R, 0, n * sizeof(float));

    if (pthread_mutex_trylock(&ins_lock) != 0) { abs_frames += n; return 0; }

    int ext = atomic_load(&sync_ext);
    spr = (double)srate * 60.0 / (atomic_load(&bpm) * 4.0);

    int c = atomic_exchange(&cmd, 0);
    if (c == 1 && !ext) { next_row = atomic_load(&cmd_row) % PAT_LEN; acc = spr; running = 1; }
    else if (c == 2)    { running = 0; kill_voices(); }

    int pr = atomic_exchange(&preview_req, 0);
    if (pr) {
        int i = (pr >> 8) - 1, note = pr & 0xff;
        if (i >= 0 && i < NINST) voice_trigger(PREVIEW, i, note, 1.0f);
    }

    void *mb = jack_port_get_buffer(midi_in, n);
    uint32_t ne = jack_midi_get_event_count(mb), ei = 0;
    jack_midi_event_t ev;
    int have = 0;
    if (ne > 0 && jack_midi_event_get(&ev, mb, ei++) == 0) have = 1;

    jack_nframes_t pos = 0;
    while (pos < n) {
        if (!ext && running && acc >= spr) { acc -= spr; step_row(); }

        jack_nframes_t end = n;
        if (have && ev.time < end) end = ev.time;
        if (end < pos) end = pos;
        if (!ext && running) {
            double rem = spr - acc;
            jack_nframes_t len = (jack_nframes_t)ceil(rem);
            if (len < 1) len = 1;
            if (pos + len < end) end = pos + len;
        }
        if (end > pos) {
            render(L, R, pos, end);
            if (!ext && running) acc += (double)(end - pos);
            pos = end;
        }
        while (have && ev.time <= pos) {
            handle_midi(&ev, ext);
            have = (ei < ne && jack_midi_event_get(&ev, mb, ei++) == 0);
        }
    }

    atomic_store(&playing, running);
    abs_frames += n;
    pthread_mutex_unlock(&ins_lock);
    return 0;
}

/* --------------------------------------------------------- sample import */

static void swap_sample(int slot, float *data, uint32_t len, uint32_t rate, const char *name)
{
    pthread_mutex_lock(&ins_lock);
    float *old = ins[slot].data;
    ins[slot].data = data;
    ins[slot].len  = len;
    ins[slot].rate = rate;
    snprintf(ins[slot].name, sizeof ins[slot].name, "%s", name);
    for (int v = 0; v < NVOICE; v++)
        if (voices[v].inst == slot) voices[v].active = 0;
    pthread_mutex_unlock(&ins_lock);
    free(old);
}

static int load_sample(int slot, const char *path, char *err, size_t errlen)
{
    SF_INFO si;
    memset(&si, 0, sizeof si);
    SNDFILE *sf = sf_open(path, SFM_READ, &si);
    if (!sf) { snprintf(err, errlen, "%s", sf_strerror(NULL)); return -1; }
    if (si.frames <= 0 || si.channels < 1) {
        sf_close(sf); snprintf(err, errlen, "empty file"); return -1;
    }
    sf_count_t want = si.frames > MAX_FRAMES ? MAX_FRAMES : si.frames;
    float *mono = malloc((size_t)want * sizeof(float));
    float *buf  = malloc(4096 * (size_t)si.channels * sizeof(float));
    if (!mono || !buf) { free(mono); free(buf); sf_close(sf); snprintf(err, errlen, "out of memory"); return -1; }

    sf_count_t total = 0;
    while (total < want) {
        sf_count_t chunk = want - total > 4096 ? 4096 : want - total;
        sf_count_t got = sf_readf_float(sf, buf, chunk);
        if (got <= 0) break;
        for (sf_count_t i = 0; i < got; i++) {
            float s = 0;
            for (int ch = 0; ch < si.channels; ch++) s += buf[i * si.channels + ch];
            mono[total + i] = s / si.channels;
        }
        total += got;
    }
    sf_close(sf);
    free(buf);
    if (total == 0) { free(mono); snprintf(err, errlen, "could not read audio"); return -1; }

    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    swap_sample(slot, mono, (uint32_t)total, (uint32_t)si.samplerate, base);
    return 0;
}

/* -------------------------------------------------------------------- UI */

enum { V_PATTERN, V_INST };

static int  view = V_PATTERN;
static int  cur_row, cur_ch, cur_field;        /* fields: 0 note 1 inst 2 volhi 3 vollo */
static int  octave = 4, cur_inst, step = 1;
static int  inst_sel, inst_param;              /* params: 0 tune 1 fine 2 vol */
static char status[128] = "";
static float peaks[PEAK_COLS];
static int  peaks_dirty = 1;
static volatile sig_atomic_t quit_flag;

static void on_sigint(int s) { (void)s; quit_flag = 1; }

static void note_str(int n, char *o)
{
    static const char *nm[12] = { "C-","C#","D-","D#","E-","F-","F#","G-","G#","A-","A#","B-" };
    if (!n) strcpy(o, "---");
    else if (n == NOTE_OFF) strcpy(o, "===");
    else snprintf(o, 8, "%s%d", nm[(n - 1) % 12], (n - 1) / 12);
}

static int key_to_note(int k, int oct)
{
    static const char lower[] = "zsxdcvgbhnjm";
    static const char upper[] = "q2w3er5t6y7ui9o0p";
    if (k <= 0 || k >= 128) return 0;
    const char *p;
    int n = 0;
    if ((p = strchr(lower, k)) && *p)      n = 1 + oct * 12 + (int)(p - lower);
    else if ((p = strchr(upper, k)) && *p) n = 1 + (oct + 1) * 12 + (int)(p - upper);
    return (n >= 1 && n <= 96) ? n : 0;
}

static int hexval(int k)
{
    if (k >= '0' && k <= '9') return k - '0';
    if (k >= 'a' && k <= 'f') return k - 'a' + 10;
    if (k >= 'A' && k <= 'F') return k - 'A' + 10;
    return -1;
}

static void audition(int inst, int note)
{
    atomic_store(&preview_req, ((inst + 1) << 8) | note);
}

static void move_row(int d) { cur_row = ((cur_row + d) % PAT_LEN + PAT_LEN) % PAT_LEN; }

static void title_bar(const char *name)
{
    char right[32], sync[24];
    int ext = atomic_load(&sync_ext);
    if (ext) snprintf(sync, sizeof sync, "MIDI %5.1f", atomic_load(&bpm_est10) / 10.0);
    else     snprintf(sync, sizeof sync, "INT");
    if (atomic_load(&playing)) snprintf(right, sizeof right, "PLAY %02X", atomic_load(&play_row));
    else                       snprintf(right, sizeof right, "STOP");

    attron(A_REVERSE);
    move(0, 0);
    for (int x = 0; x < COLS; x++) addch(' ');
    mvprintw(0, 1, "XTRACK  %-11s BPM:%3d  SYNC:%-10s OCT:%d  INS:%d  STEP:%d",
             name, atomic_load(&bpm), sync, octave, cur_inst + 1, step);
    mvprintw(0, COLS - (int)strlen(right) - 1, "%s", right);
    attroff(A_REVERSE);
}

static void footer(const char *help)
{
    if (status[0]) mvprintw(LINES - 2, 1, "%.*s", COLS - 2, status);
    attron(A_DIM);
    mvprintw(LINES - 1, 1, "%.*s", COLS - 2, help);
    attroff(A_DIM);
}

static void draw_pattern(void)
{
    static const int foff[4] = { 0, 4, 7, 8 }, flen[4] = { 3, 2, 1, 1 };
    title_bar("PATTERN");

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(1, 0, "ROW");
    for (int c = 0; c < NCHAN; c++) mvprintw(1, 5 + c * 12, "CH%d  nte in vl", c + 1);
    attroff(COLOR_PAIR(4) | A_BOLD);

    int vis = LINES - 4;
    int top = cur_row - vis / 2;
    if (top > PAT_LEN - vis) top = PAT_LEN - vis;
    if (top < 0) top = 0;
    int pl = atomic_load(&playing), prow = atomic_load(&play_row);

    for (int i = 0; i < vis && top + i < PAT_LEN; i++) {
        int r = top + i, y = 2 + i;
        int attr = (pl && r == prow) ? (COLOR_PAIR(2) | A_BOLD)
                 : (r % 4 == 0 ? COLOR_PAIR(3) : A_NORMAL);
        if (r == cur_row) attr |= A_BOLD;
        attron(attr);
        move(y, 0);
        for (int x = 0; x < COLS; x++) addch(' ');
        mvprintw(y, 0, "%02X", r);
        for (int c = 0; c < NCHAN; c++) {
            Cell *ce = &pat[r][c];
            char nn[8], ii[8], vv[8], s[32];
            note_str(ce->note, nn);
            if (ce->inst) snprintf(ii, sizeof ii, "%02X", ce->inst); else strcpy(ii, "--");
            if (ce->vol >= 0) snprintf(vv, sizeof vv, "%02X", ce->vol); else strcpy(vv, "--");
            snprintf(s, sizeof s, "%s %s %s", nn, ii, vv);
            int x = 5 + c * 12;
            mvaddstr(y, x - 2, "|");
            mvaddstr(y, x, s);
            if (r == cur_row && c == cur_ch) {
                attron(A_REVERSE);
                mvaddnstr(y, x + foff[cur_field], s + foff[cur_field], flen[cur_field]);
                attroff(A_REVERSE);
            }
        }
        attroff(attr);
    }
    footer("F1/F2 view  Space play  Enter play@cursor  []inst ()step  Del clear  1=off  F10 quit");
}

static void compute_peaks(void)
{
    Inst *in = &ins[inst_sel];
    memset(peaks, 0, sizeof peaks);
    if (in->data) {
        for (int b = 0; b < PEAK_COLS; b++) {
            uint64_t s = (uint64_t)b * in->len / PEAK_COLS;
            uint64_t e = (uint64_t)(b + 1) * in->len / PEAK_COLS;
            float m = 0;
            for (uint64_t i = s; i < e; i++) { float a = fabsf(in->data[i]); if (a > m) m = a; }
            peaks[b] = m > 1.0f ? 1.0f : m;
        }
    }
    peaks_dirty = 0;
}

static void draw_inst(void)
{
    title_bar("INSTRUMENTS");
    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(1, 1, "#  NAME                     LEN       RATE   TUNE   FINE   VOL");
    attroff(COLOR_PAIR(4) | A_BOLD);

    static const int px[3] = { 49, 56, 63 }, pw[3] = { 3, 4, 2 };
    for (int i = 0; i < NINST; i++) {
        Inst *in = &ins[i];
        int y = 2 + i;
        char tune[8], fine[8], vol[8];
        snprintf(tune, sizeof tune, "%+3d", in->tune);
        snprintf(fine, sizeof fine, "%+4d", in->fine);
        snprintf(vol,  sizeof vol,  "%02X", in->vol);
        int sel = (i == inst_sel);
        if (sel) attron(A_BOLD);
        mvprintw(y, 0, "%c%d  %-22.22s", sel ? '>' : ' ', i + 1, in->data ? in->name : "(empty)");
        if (in->data) {
            mvprintw(y, 28, "%6.2fs", (double)in->len / in->rate);
            mvprintw(y, 38, "%6u", in->rate);
        }
        mvaddstr(y, px[0], tune);
        mvaddstr(y, px[1], fine);
        mvaddstr(y, px[2], vol);
        if (sel) {
            const char *txt[3] = { tune, fine, vol };
            attron(A_REVERSE);
            mvaddnstr(y, px[inst_param], txt[inst_param], pw[inst_param]);
            attroff(A_REVERSE);
        }
        if (sel) attroff(A_BOLD);
    }

    if (peaks_dirty) compute_peaks();
    mvprintw(11, 1, "WAVEFORM");
    int cy = 15, w = COLS - 4 < PEAK_COLS ? COLS - 4 : PEAK_COLS;
    mvhline(cy, 2, '-', w);
    for (int x = 0; x < w; x++) {
        int h = (int)lrintf(peaks[x] * 3.0f);
        for (int dy = -h; dy <= h; dy++) mvaddch(cy + dy, 2 + x, '#');
    }
    footer("Up/Dn slot  Tab param  Lt/Rt adjust (PgUp/Dn x big)  L/Enter load  Del unload  keys=audition");
}

static void prompt_load(void)
{
    char buf[512], path[640], err[96];
    move(LINES - 2, 0); clrtoeol();
    printw(" Load sample into slot %d (empty = cancel): ", inst_sel + 1);
    echo(); curs_set(1); timeout(-1);
    getnstr(buf, sizeof buf - 1);
    noecho(); curs_set(0); timeout(33);

    char *p = buf;
    while (isspace((unsigned char)*p)) p++;
    size_t l = strlen(p);
    while (l && isspace((unsigned char)p[l - 1])) p[--l] = 0;
    if (!*p) { status[0] = 0; return; }
    if (p[0] == '~' && (p[1] == '/' || p[1] == 0) && getenv("HOME"))
        snprintf(path, sizeof path, "%s%s", getenv("HOME"), p + 1);
    else
        snprintf(path, sizeof path, "%s", p);

    if (load_sample(inst_sel, path, err, sizeof err) == 0) {
        snprintf(status, sizeof status, "Loaded into slot %d: %s", inst_sel + 1, ins[inst_sel].name);
        peaks_dirty = 1;
    } else {
        snprintf(status, sizeof status, "Load failed: %s", err);
    }
}

static void transport_toggle(int from_cursor)
{
    if (atomic_load(&sync_ext)) {
        snprintf(status, sizeof status, "MIDI sync on: transport follows MIDI Start/Stop (F9 to switch)");
        return;
    }
    if (atomic_load(&playing) && !from_cursor) {
        atomic_store(&cmd, 2);
    } else {
        atomic_store(&cmd_row, from_cursor ? cur_row : 0);
        atomic_store(&cmd, 1);
    }
}

static void adjust_param(int d)
{
    Inst *in = &ins[inst_sel];
    switch (inst_param) {
    case 0: in->tune += d; if (in->tune < -48) in->tune = -48; if (in->tune > 48) in->tune = 48; break;
    case 1: in->fine += d; if (in->fine < -99) in->fine = -99; if (in->fine > 99) in->fine = 99; break;
    case 2: in->vol  += d; if (in->vol < 0) in->vol = 0; if (in->vol > 255) in->vol = 255; break;
    }
}

static void inst_key(int k)
{
    switch (k) {
    case KEY_UP:    inst_sel = (inst_sel + NINST - 1) % NINST; cur_inst = inst_sel; peaks_dirty = 1; return;
    case KEY_DOWN:  inst_sel = (inst_sel + 1) % NINST;         cur_inst = inst_sel; peaks_dirty = 1; return;
    case '\t':      inst_param = (inst_param + 1) % 3; return;
    case KEY_BTAB:  inst_param = (inst_param + 2) % 3; return;
    case KEY_LEFT:  adjust_param(-1); return;
    case KEY_RIGHT: adjust_param(+1); return;
    case KEY_PPAGE: adjust_param(inst_param == 0 ? 12 : inst_param == 1 ? 10 : 16); return;
    case KEY_NPAGE: adjust_param(inst_param == 0 ? -12 : inst_param == 1 ? -10 : -16); return;
    case 'l': case 'L': case '\n': case KEY_ENTER: prompt_load(); return;
    case KEY_DC:
        swap_sample(inst_sel, NULL, 0, 0, "");
        peaks_dirty = 1;
        snprintf(status, sizeof status, "Slot %d cleared", inst_sel + 1);
        return;
    }
    int n = key_to_note(k, octave);
    if (n) audition(inst_sel, n);
}

static void pattern_key(int k)
{
    Cell *ce = &pat[cur_row][cur_ch];
    switch (k) {
    case KEY_UP:    move_row(-1); return;
    case KEY_DOWN:  move_row(1);  return;
    case KEY_PPAGE: move_row(-16); return;
    case KEY_NPAGE: move_row(16);  return;
    case KEY_HOME:  cur_row = 0; return;
    case KEY_END:   cur_row = PAT_LEN - 1; return;
    case KEY_LEFT:
        if (--cur_field < 0) { cur_field = 3; cur_ch = (cur_ch + NCHAN - 1) % NCHAN; }
        return;
    case KEY_RIGHT:
        if (++cur_field > 3) { cur_field = 0; cur_ch = (cur_ch + 1) % NCHAN; }
        return;
    case '\t':     cur_ch = (cur_ch + 1) % NCHAN; cur_field = 0; return;
    case KEY_BTAB: cur_ch = (cur_ch + NCHAN - 1) % NCHAN; cur_field = 0; return;
    case '[': cur_inst = (cur_inst + NINST - 1) % NINST; return;
    case ']': cur_inst = (cur_inst + 1) % NINST; return;
    case '(': if (step > 0)  step--; return;
    case ')': if (step < 16) step++; return;
    case '\n': case KEY_ENTER: transport_toggle(1); return;
    case KEY_DC:
        if (cur_field == 0) { ce->note = 0; ce->inst = 0; ce->vol = -1; }
        else if (cur_field == 1) ce->inst = 0;
        else ce->vol = -1;
        move_row(step);
        return;
    }

    if (cur_field == 0) {
        if (k == '1') { ce->note = NOTE_OFF; ce->inst = 0; move_row(step); }
        else {
            int n = key_to_note(k, octave);
            if (n) {
                ce->note = n;
                ce->inst = cur_inst + 1;
                audition(cur_inst, n);
                move_row(step);
            }
        }
    } else if (cur_field == 1) {
        if (k >= '1' && k <= '0' + NINST) {
            ce->inst = k - '0';
            cur_inst = k - '1';
            move_row(step);
        }
    } else {
        int d = hexval(k);
        if (d >= 0) {
            int v = ce->vol < 0 ? 0 : ce->vol;
            if (cur_field == 2) { v = (d << 4) | (v & 15); cur_field = 3; }
            else                { v = (v & 0xF0) | d;      cur_field = 2; move_row(step); }
            ce->vol = v;
        }
    }
}

static void global_key(int k, int *handled)
{
    *handled = 1;
    switch (k) {
    case KEY_F(1): view = V_PATTERN; break;
    case KEY_F(2): view = V_INST; peaks_dirty = 1; break;
    case KEY_F(3): if (octave > 0) octave--; break;
    case KEY_F(4): if (octave < 7) octave++; break;
    case KEY_F(9): {
        int e = !atomic_load(&sync_ext);
        atomic_store(&sync_ext, e);
        atomic_store(&cmd, 2);
        snprintf(status, sizeof status, e ? "Sync: MIDI clock (waiting for Start)" : "Sync: internal");
        break;
    }
    case KEY_F(10): quit_flag = 1; break;
    case ' ': transport_toggle(0); break;
    case '<': { int b = atomic_load(&bpm) - 1; atomic_store(&bpm, b < 32 ? 32 : b); break; }
    case '>': { int b = atomic_load(&bpm) + 1; atomic_store(&bpm, b > 300 ? 300 : b); break; }
    default: *handled = 0;
    }
}

/* ------------------------------------------------------------------ main */

static void demo_pattern(void)
{
    for (int r = 0; r < PAT_LEN; r++)
        for (int c = 0; c < NCHAN; c++) pat[r][c] = (Cell){ 0, 0, -1 };
    for (int r = 0; r < PAT_LEN; r += 4) pat[r][0] = (Cell){ 49, 1, 0xFF };
    for (int r = 4; r < PAT_LEN; r += 8) pat[r][1] = (Cell){ 49, 2, 0xC0 };
    for (int r = 2; r < PAT_LEN; r += 4) pat[r][2] = (Cell){ 49, 3, 0x60 };
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    for (int i = 0; i < NINST; i++) ins[i].vol = 255;
    demo_pattern();

    jack_status_t jst;
    client = jack_client_open("xtrack", JackNullOption, &jst);
    if (!client) {
        fprintf(stderr, "xtrack: cannot connect to JACK (status 0x%x). Is jackd / pipewire-jack running?\n", jst);
        return 1;
    }
    srate = jack_get_sample_rate(client);
    out_l   = jack_port_register(client, "out_l",   JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    out_r   = jack_port_register(client, "out_r",   JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    midi_in = jack_port_register(client, "midi_in", JACK_DEFAULT_MIDI_TYPE,  JackPortIsInput,  0);
    jack_set_process_callback(client, process, NULL);

    for (int i = 1; i < argc && i <= NINST; i++) {
        char err[96];
        if (load_sample(i - 1, argv[i], err, sizeof err) != 0)
            fprintf(stderr, "xtrack: %s: %s\n", argv[i], err);
    }

    if (jack_activate(client)) { fprintf(stderr, "xtrack: cannot activate JACK client\n"); return 1; }

    const char **ports = jack_get_ports(client, NULL, JACK_DEFAULT_AUDIO_TYPE,
                                        JackPortIsPhysical | JackPortIsInput);
    if (ports) {
        if (ports[0]) jack_connect(client, jack_port_name(out_l), ports[0]);
        if (ports[0] && ports[1]) jack_connect(client, jack_port_name(out_r), ports[1]);
        jack_free(ports);
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sigint;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    initscr();
    cbreak(); noecho(); keypad(stdscr, TRUE); curs_set(0); timeout(33);
    if (has_colors()) {
        start_color(); use_default_colors();
        init_pair(2, COLOR_BLACK, COLOR_GREEN);   /* play head */
        init_pair(3, COLOR_YELLOW, -1);           /* beat rows */
        init_pair(4, COLOR_CYAN, -1);             /* headers   */
    }

    while (!quit_flag) {
        erase();
        if (LINES < 24 || COLS < 80) {
            mvprintw(0, 0, "Terminal too small: need 80x24 (have %dx%d)", COLS, LINES);
        } else if (view == V_PATTERN) draw_pattern();
        else draw_inst();
        refresh();

        int k = getch();
        if (k == ERR) continue;
        if (k == KEY_RESIZE) continue;
        if (k != ' ' && status[0] && k != KEY_F(9)) status[0] = 0;

        int handled;
        global_key(k, &handled);
        if (handled) continue;
        if (view == V_PATTERN) pattern_key(k); else inst_key(k);
    }

    endwin();
    jack_client_close(client);
    return 0;
}
