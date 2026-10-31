/*
 * xtrack - small ncurses music tracker
 *
 *   - fits in an 80x24 terminal
 *   - JACK stereo audio out, JACK MIDI in (clock / start / stop / continue / SPP)
 *   - 4 tracker channels, 64-row pattern, 8 instrument slots
 *   - cell = note, instrument number, 2-digit hex volume
 *   - instruments: sample import (anything libsndfile reads), tune (semitones),
 *     fine tune (cents), volume, loop (off / forward / ping-pong, start+end),
 *     ADSR amplitude envelope
 *   - project save/load (text file, references samples by path)
 *
 * Usage:  ./xtrack [project.xtp] [sample1.wav ... sample8.wav]
 *
 * Keys (global)
 *   F1 pattern view        F2 instrument view
 *   F3/F4 octave -/+       F5 save project   F6 load project
 *   F7 edit mode on/off (off = pattern is read-only; notes just audition)
 *   F9 sync INT <-> MIDI   F10 or Ctrl-C quit
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
 *   Up/Down select slot   Tab select param: tune fine vol loop lstart lend
 *                         att dec sus rel (amplitude envelope, times in ms)
 *   Left/Right adjust     PgUp/PgDn big adjust    , .  1-frame adjust (loop points)
 *   L or Enter load sample    Del unload
 *   piano keys audition the selected instrument, 1 stops the audition
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
#include <errno.h>
#include <limits.h>
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
#define PATHLEN    640
#define MAX_ENV_MS 5000
#define PREVIEW_SECS 0.5              /* audition note length */

enum { LOOP_OFF, LOOP_FWD, LOOP_PING };

/* ------------------------------------------------------------------ data */

typedef struct { uint8_t note, inst; int16_t vol; } Cell;   /* vol -1 = none */

typedef struct {
    float   *data;               /* mono float */
    uint32_t len, rate;
    int      tune;               /* semitones  -48..48 */
    int      fine;               /* cents      -99..99 */
    int      vol;                /* 0..255 */
    int      loop;               /* LOOP_* */
    uint32_t loop_start, loop_end;   /* frames, end exclusive */
    int      att_ms, dec_ms; /* envelope: attack / decay time (ms) */
    int      sus;            /* sustain level 0..255 */
    int      rel_ms;         /* release time (ms) */
    char     name[24];
    char     path[PATHLEN];      /* source file (UI thread only) */
} Inst;

enum { ENV_ATK, ENV_DEC, ENV_SUS, ENV_REL };

typedef struct {
    int active, inst, dir, stage;
    double pos, step;
    float gain, level, rel_step;
} Voice;

static Cell  pat[PAT_LEN][NCHAN];
static Inst  ins[NINST];
static Voice voices[NVOICE];
static const float pan[NVOICE] = { 0.25f, 0.75f, 0.75f, 0.25f, 0.5f };

/* instrument table / pattern swaps are guarded by this; audio thread only try-locks */
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
static int64_t  preview_left;          /* frames remaining on audition voice */

/* ----------------------------------------------------------------- audio */

static void kill_voices(void)
{
    for (int v = 0; v < NCHAN; v++) voices[v].active = 0;
}

static void voice_trigger(int v, int inst, int note, float vol)
{
    Voice *vc = &voices[v];
    vc->active = 0;                       /* a new note always cuts the previous one */
    Inst *in = &ins[inst];
    if (!in->data || !in->len) return;
    double semis = (note - BASE_NOTE) + in->tune + in->fine / 100.0;
    vc->inst = inst;
    vc->pos  = 0;
    vc->dir  = 1;
    vc->step = ((double)in->rate / (double)srate) * pow(2.0, semis / 12.0);
    vc->gain = vol * (in->vol / 255.0f);
    if (in->att_ms > 0) { vc->level = 0.0f; vc->stage = ENV_ATK; }
    else                { vc->level = 1.0f; vc->stage = ENV_DEC; }
    vc->active = 1;
}

/* start the release stage (note-off); min 2 ms so cuts never click */
static void voice_release(Voice *vc)
{
    if (!vc->active || vc->stage == ENV_REL) return;
    if (vc->level <= 0.0f) { vc->active = 0; return; }
    float frames = (float)ins[vc->inst].rel_ms * (float)srate / 1000.0f;
    float minf   = (float)srate * 0.002f;
    if (frames < minf) frames = minf;
    vc->rel_step = vc->level / frames;
    vc->stage = ENV_REL;
}

static void step_row(void)
{
    int r = next_row;
    next_row = (r + 1) % PAT_LEN;
    atomic_store(&play_row, r);

    for (int c = 0; c < NCHAN; c++) {
        Cell *ce = &pat[r][c];
        if (ce->note == NOTE_OFF) {
            voice_release(&voices[c]);
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
        const float *d = in->data;
        if (!d) { vc->active = 0; continue; }

        uint32_t len = in->len;
        int      loop = in->loop;
        uint32_t le = in->loop_end > len ? len : in->loop_end;
        uint32_t ls = in->loop_start;
        if (le < 2 || ls + 2 > le) loop = LOOP_OFF;

        float sus     = in->sus / 255.0f;
        float atk_inc = in->att_ms > 0 ? 1000.0f / ((float)in->att_ms * (float)srate) : 1.0f;
        float dec_dec = in->dec_ms > 0 ? (1.0f - sus) * 1000.0f / ((float)in->dec_ms * (float)srate) : 1.0f;

        float pr = pan[v], pl = 1.0f - pr;
        float g  = vc->gain * MIX_GAIN;

        for (jack_nframes_t i = from; i < to; i++) {
            if (vc->pos < 0) vc->pos = 0;
            uint32_t idx = (uint32_t)vc->pos;
            if (idx >= len) { vc->active = 0; break; }

            /* audition voice: auto note-off after its hold time */
            if (v == PREVIEW && preview_left > 0 && --preview_left == 0) voice_release(vc);

            float    fr = (float)(vc->pos - idx);
            uint32_t i1 = idx + 1;
            if (loop && i1 >= le) i1 = (loop == LOOP_FWD) ? ls : le - 1;
            float s0 = d[idx];
            float s1 = (i1 < len) ? d[i1] : 0.0f;
            float s  = (s0 + (s1 - s0) * fr) * g * vc->level;
            L[i] += s * pl;
            R[i] += s * pr;

            switch (vc->stage) {
            case ENV_ATK:
                vc->level += atk_inc;
                if (vc->level >= 1.0f) { vc->level = 1.0f; vc->stage = ENV_DEC; }
                break;
            case ENV_DEC:
                vc->level -= dec_dec;
                if (vc->level <= sus) { vc->level = sus; vc->stage = ENV_SUS; }
                break;
            case ENV_SUS:
                vc->level = sus;
                break;
            case ENV_REL:
                vc->level -= vc->rel_step;
                if (vc->level <= 0.0f) vc->active = 0;
                break;
            }
            if (!vc->active) break;

            vc->pos += vc->step * vc->dir;

            if (loop == LOOP_FWD) {
                if (vc->pos >= le)
                    vc->pos = ls + fmod(vc->pos - ls, (double)(le - ls));
            } else if (loop == LOOP_PING) {
                if (vc->dir > 0 && vc->pos >= le)      { vc->pos = 2.0 * le - vc->pos; vc->dir = -1; }
                else if (vc->dir < 0 && vc->pos < ls)  { vc->pos = 2.0 * ls - vc->pos; vc->dir = 1;  }
                if (vc->pos < ls)       vc->pos = ls;
                if (vc->pos > le - 1.0) vc->pos = le - 1.0;
            }
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
        if (note == NOTE_OFF)                 voice_release(&voices[PREVIEW]);
        else if (i >= 0 && i < NINST) {
            voice_trigger(PREVIEW, i, note, 1.0f);
            if (voices[PREVIEW].active)       /* hold long enough to hear attack+decay */
                preview_left = (int64_t)(srate * (PREVIEW_SECS + (ins[i].att_ms + ins[i].dec_ms) / 1000.0));
        }
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
    ins[slot].loop_start = 0;
    ins[slot].loop_end   = len;
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

    char rp[PATH_MAX];
    snprintf(ins[slot].path, sizeof ins[slot].path, "%s", realpath(path, rp) ? rp : path);
    return 0;
}

/* --------------------------------------------------------------- project */

static char proj_path[PATHLEN] = "";

static int project_save(const char *path, char *err, size_t el)
{
    char tmp[PATHLEN + 8];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) { snprintf(err, el, "%s", strerror(errno)); return -1; }

    fprintf(f, "XTRACK 1\nbpm %d\n", atomic_load(&bpm));
    for (int i = 0; i < NINST; i++) {
        Inst *in = &ins[i];
        fprintf(f, "inst %d tune=%d fine=%d vol=%d loop=%d ls=%u le=%u a=%d d=%d s=%d r=%d path=%s\n",
                i + 1, in->tune, in->fine, in->vol, in->loop,
                (unsigned)in->loop_start, (unsigned)in->loop_end,
                in->att_ms, in->dec_ms, in->sus, in->rel_ms, in->path);
    }
    for (int r = 0; r < PAT_LEN; r++) {
        int any = 0;
        for (int c = 0; c < NCHAN; c++)
            if (pat[r][c].note || pat[r][c].inst || pat[r][c].vol >= 0) any = 1;
        if (!any) continue;
        fprintf(f, "row %d", r);
        for (int c = 0; c < NCHAN; c++)
            fprintf(f, " %d:%d:%d", pat[r][c].note, pat[r][c].inst, pat[r][c].vol);
        fputc('\n', f);
    }
    int bad = ferror(f);
    if (fclose(f)) bad = 1;
    if (bad || rename(tmp, path) != 0) {
        snprintf(err, el, "%s", strerror(errno ? errno : EIO));
        remove(tmp);
        return -1;
    }
    return 0;
}

static int project_load(const char *file, char *err, size_t el, int *missing)
{
    struct Cfg { int have, tune, fine, vol, loop, att, dec, sus, rel; unsigned ls, le; char path[PATHLEN]; } cfg[NINST];
    Cell tmp[PAT_LEN][NCHAN];
    char line[1024];
    int newbpm = 0;

    *missing = 0;
    FILE *f = fopen(file, "r");
    if (!f) { snprintf(err, el, "%s", strerror(errno)); return -1; }
    if (!fgets(line, sizeof line, f) || strncmp(line, "XTRACK", 6) != 0) {
        fclose(f); snprintf(err, el, "not an xtrack project"); return -1;
    }

    memset(cfg, 0, sizeof cfg);
    for (int i = 0; i < NINST; i++) { cfg[i].vol = 255; cfg[i].sus = 255; }
    for (int r = 0; r < PAT_LEN; r++)
        for (int c = 0; c < NCHAN; c++) tmp[r][c] = (Cell){ 0, 0, -1 };

    while (fgets(line, sizeof line, f)) {
        size_t l = strlen(line);
        while (l && (line[l - 1] == '\n' || line[l - 1] == '\r')) line[--l] = 0;

        int a;
        if (sscanf(line, "bpm %d", &a) == 1) {
            newbpm = a;
        } else if (!strncmp(line, "inst ", 5)) {
            char path[PATHLEN] = "";
            char *p = strstr(line, " path=");
            if (p) { snprintf(path, sizeof path, "%s", p + 6); *p = 0; }
            int n, tu, fi, vo, lo;
            unsigned ls, le;
            if (sscanf(line, "inst %d tune=%d fine=%d vol=%d loop=%d ls=%u le=%u",
                       &n, &tu, &fi, &vo, &lo, &ls, &le) == 7 && n >= 1 && n <= NINST) {
                struct Cfg *c = &cfg[n - 1];
                c->have = 1;
                c->tune = tu < -48 ? -48 : tu > 48 ? 48 : tu;
                c->fine = fi < -99 ? -99 : fi > 99 ? 99 : fi;
                c->vol  = vo < 0 ? 0 : vo > 255 ? 255 : vo;
                c->loop = (lo >= 0 && lo <= LOOP_PING) ? lo : 0;
                c->ls = ls; c->le = le;
                int ea, ed, es, er;      /* envelope fields are optional (older files) */
                if (sscanf(line, "inst %*d tune=%*d fine=%*d vol=%*d loop=%*d ls=%*u le=%*u a=%d d=%d s=%d r=%d",
                           &ea, &ed, &es, &er) == 4) {
                    c->att = ea < 0 ? 0 : ea > MAX_ENV_MS ? MAX_ENV_MS : ea;
                    c->dec = ed < 0 ? 0 : ed > MAX_ENV_MS ? MAX_ENV_MS : ed;
                    c->sus = es < 0 ? 0 : es > 255 ? 255 : es;
                    c->rel = er < 0 ? 0 : er > MAX_ENV_MS ? MAX_ENV_MS : er;
                }
                snprintf(c->path, sizeof c->path, "%s", path);
            }
        } else if (!strncmp(line, "row ", 4)) {
            int r, off = 0;
            if (sscanf(line, "row %d%n", &r, &off) == 1 && r >= 0 && r < PAT_LEN) {
                char *q = line + off;
                for (int c = 0; c < NCHAN; c++) {
                    int n, i, v, used = 0;
                    if (sscanf(q, " %d:%d:%d%n", &n, &i, &v, &used) != 3) break;
                    q += used;
                    if (n < 0 || n > NOTE_OFF || i < 0 || i > NINST || v < -1 || v > 255) continue;
                    tmp[r][c] = (Cell){ (uint8_t)n, (uint8_t)i, (int16_t)v };
                }
            }
        }
    }
    fclose(f);

    char dir[PATHLEN];
    snprintf(dir, sizeof dir, "%s", file);
    char *sl = strrchr(dir, '/');
    if (sl) *sl = 0; else strcpy(dir, ".");

    atomic_store(&cmd, 2);
    pthread_mutex_lock(&ins_lock);
    memcpy(pat, tmp, sizeof pat);
    pthread_mutex_unlock(&ins_lock);
    if (newbpm >= 32 && newbpm <= 300) atomic_store(&bpm, newbpm);

    for (int i = 0; i < NINST; i++) {
        struct Cfg *c = &cfg[i];
        Inst *in = &ins[i];
        char e2[96];

        swap_sample(i, NULL, 0, 0, "");
        snprintf(in->path, sizeof in->path, "%s", c->path);
        if (c->path[0]) {
            int rc = load_sample(i, c->path, e2, sizeof e2);
            if (rc) {                                   /* fall back to project dir */
                const char *b = strrchr(c->path, '/');
                char alt[PATHLEN + 8];
                snprintf(alt, sizeof alt, "%s/%s", dir, b ? b + 1 : c->path);
                rc = load_sample(i, alt, e2, sizeof e2);
            }
            if (rc) (*missing)++;
        }
        in->tune = c->tune; in->fine = c->fine; in->vol = c->vol; in->loop = c->loop;
        in->att_ms = c->att; in->dec_ms = c->dec; in->sus = c->sus; in->rel_ms = c->rel;
        if (in->data) {
            uint32_t le = c->le;
            if (le == 0 || le > in->len) le = in->len;
            uint32_t ls = c->ls;
            if (le < 2 || ls + 2 > le) ls = 0;
            in->loop_start = ls;
            in->loop_end   = le;
        } else {
            in->loop_start = c->ls;                     /* keep so a re-save preserves it */
            in->loop_end   = c->le;
        }
    }

    snprintf(proj_path, sizeof proj_path, "%s", file);
    return 0;
}

/* -------------------------------------------------------------------- UI */

enum { V_PATTERN, V_INST };
enum { P_TUNE, P_FINE, P_VOL, P_LOOP, P_LSTART, P_LEND, P_ATT, P_DEC, P_SUS, P_REL, NPARAM };

static int  view = V_PATTERN;
static int  cur_row, cur_ch, cur_field;        /* fields: 0 note 1 inst 2 volhi 3 vollo */
static int  octave = 4, cur_inst, step = 1;
static int  inst_sel, inst_param;
static int  edit_mode = 0;                     /* 0 = view/audition only, 1 = pattern edits allowed */
static char status[160] = "";
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

static const char *base_name(const char *p)
{
    const char *b = strrchr(p, '/');
    return b ? b + 1 : p;
}

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

static void footer(const char *help1, const char *help2)
{
    if (status[0]) mvprintw(LINES - 2, 1, "%.*s", COLS - 2, status);
    else { attron(A_DIM); mvprintw(LINES - 2, 1, "%.*s", COLS - 2, help2); attroff(A_DIM); }
    attron(A_DIM);
    mvprintw(LINES - 1, 1, "%.*s", COLS - 2, help1);
    attroff(A_DIM);
}

#define HELP_GLOBAL "F1 pattern F2 instr F3/F4 oct F5 save F6 load F7 edit F9 sync F10 quit"

static void draw_pattern(void)
{
    static const int foff[4] = { 0, 4, 7, 8 }, flen[4] = { 3, 2, 1, 1 };
    title_bar("PATTERN");
    if (edit_mode) { attron(COLOR_PAIR(5) | A_BOLD); mvaddstr(0, 67, "EDIT"); attroff(COLOR_PAIR(5) | A_BOLD); }
    else           { attron(A_REVERSE);              mvaddstr(0, 67, "VIEW"); attroff(A_REVERSE); }

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(1, 0, "ROW");
    for (int c = 0; c < NCHAN; c++) mvprintw(1, 5 + c * 12, "CH%d  nte in vl", c + 1);
    attroff(COLOR_PAIR(4) | A_BOLD);
    attron(A_DIM);
    mvprintw(1, 58, "%.20s", proj_path[0] ? base_name(proj_path) : "untitled");
    attroff(A_DIM);

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
    footer(HELP_GLOBAL,
           "Space play  Enter play@cursor  [ ]inst  ( )step  Del clear  1=note off  <>bpm");
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
    static const char *loopname[3] = { "OFF", "FWD", "PING" };
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
        const char *nm = in->data ? in->name : (in->path[0] ? "(missing)" : "(empty)");
        if (sel) attron(A_BOLD);
        mvprintw(y, 0, "%c%d  %-22.22s", sel ? '>' : ' ', i + 1, nm);
        if (in->data) {
            mvprintw(y, 28, "%6.2fs", (double)in->len / in->rate);
            mvprintw(y, 38, "%6u", in->rate);
            if (in->loop) mvaddstr(y, 45, in->loop == LOOP_FWD ? "L" : "P");
        }
        mvaddstr(y, px[0], tune);
        mvaddstr(y, px[1], fine);
        mvaddstr(y, px[2], vol);
        if (sel && inst_param <= P_VOL) {
            const char *txt[3] = { tune, fine, vol };
            attron(A_REVERSE);
            mvaddnstr(y, px[inst_param], txt[inst_param], pw[inst_param]);
            attroff(A_REVERSE);
        }
        if (sel) attroff(A_BOLD);
    }

    /* loop line for selected instrument */
    Inst *s = &ins[inst_sel];
    char lm[8], ls[16], le[16];
    snprintf(lm, sizeof lm, "%-4s", loopname[s->loop]);
    snprintf(ls, sizeof ls, "%8u", (unsigned)s->loop_start);
    snprintf(le, sizeof le, "%8u", (unsigned)s->loop_end);
    attron(COLOR_PAIR(4));
    mvaddstr(11, 1, "LOOP:");  mvaddstr(11, 12, "START:");
    mvaddstr(11, 28, "END:");  mvaddstr(11, 42, "LEN:");
    attroff(COLOR_PAIR(4));
    mvaddstr(11, 6, lm);  mvaddstr(11, 18, ls);  mvaddstr(11, 32, le);
    mvprintw(11, 46, "%8u", (unsigned)s->len);
    attron(A_REVERSE);
    if (inst_param == P_LOOP)   mvaddstr(11, 6, lm);
    if (inst_param == P_LSTART) mvaddstr(11, 18, ls);
    if (inst_param == P_LEND)   mvaddstr(11, 32, le);
    attroff(A_REVERSE);

    if (peaks_dirty) compute_peaks();
    {
        char ea[8], ed[8], es[8], er[8];
        snprintf(ea, sizeof ea, "%4d", s->att_ms);
        snprintf(ed, sizeof ed, "%4d", s->dec_ms);
        snprintf(es, sizeof es, "%02X", s->sus);
        snprintf(er, sizeof er, "%4d", s->rel_ms);
        attron(COLOR_PAIR(4));
        mvaddstr(12, 1, "ATT:");  mvaddstr(12, 14, "DEC:");
        mvaddstr(12, 27, "SUS:"); mvaddstr(12, 36, "REL:");
        attroff(COLOR_PAIR(4));
        mvaddstr(12, 5, ea);  mvaddstr(12, 10, "ms");
        mvaddstr(12, 18, ed); mvaddstr(12, 23, "ms");
        mvaddstr(12, 31, es);
        mvaddstr(12, 40, er); mvaddstr(12, 45, "ms");
        attron(A_REVERSE);
        if (inst_param == P_ATT) mvaddstr(12, 5, ea);
        if (inst_param == P_DEC) mvaddstr(12, 18, ed);
        if (inst_param == P_SUS) mvaddstr(12, 31, es);
        if (inst_param == P_REL) mvaddstr(12, 40, er);
        attroff(A_REVERSE);
    }
    int cy = 16, w = COLS - 4 < PEAK_COLS ? COLS - 4 : PEAK_COLS;
    mvhline(cy, 2, '-', w);
    for (int x = 0; x < w; x++) {
        int h = (int)lrintf(peaks[x] * 3.0f);
        for (int dy = -h; dy <= h; dy++) mvaddch(cy + dy, 2 + x, '#');
    }
    if (s->data && s->loop && s->len) {
        int xs = (int)((uint64_t)s->loop_start * PEAK_COLS / s->len);
        int xe = (int)((uint64_t)s->loop_end   * PEAK_COLS / s->len);
        if (xs > w - 1) xs = w - 1;
        if (xe > w - 1) xe = w - 1;
        for (int x = xs; x <= xe; x++) mvaddch(20, 2 + x, '=');
        mvaddch(20, 2 + xs, '[');
        mvaddch(20, 2 + xe, ']');
    }
    footer(HELP_GLOBAL,
           "Up/Dn slot Tab param Lt/Rt adj PgUp/Dn big ,/. fine L load Del clr 1=stop");
}

/* ---- prompts / project actions */

static void clean_path(char *s, size_t n)
{
    char *p = s;
    while (isspace((unsigned char)*p)) p++;
    size_t l = strlen(p);
    while (l && isspace((unsigned char)p[l - 1])) p[--l] = 0;
    char tmp[PATHLEN];
    if (p[0] == '~' && (p[1] == '/' || p[1] == 0) && getenv("HOME"))
        snprintf(tmp, sizeof tmp, "%s%s", getenv("HOME"), p + 1);
    else
        snprintf(tmp, sizeof tmp, "%s", p);
    snprintf(s, n, "%s", tmp);
}

static void ensure_ext(char *path, size_t n)
{
    if (!strchr(base_name(path), '.') && strlen(path) + 5 < n) strcat(path, ".xtp");
}

/* returns 1 if non-empty line entered */
static int prompt_line(const char *msg, char *out, size_t n)
{
    move(LINES - 2, 0); clrtoeol();
    printw(" %s", msg);
    echo(); curs_set(1); timeout(-1);
    out[0] = 0;
    getnstr(out, (int)n - 1);
    noecho(); curs_set(0); timeout(33);
    clean_path(out, n);
    return out[0] != 0;
}

static void prompt_load_sample(void)
{
    char path[PATHLEN], msg[96], err[96];
    snprintf(msg, sizeof msg, "Load sample into slot %d (empty = cancel): ", inst_sel + 1);
    if (!prompt_line(msg, path, sizeof path)) { status[0] = 0; return; }

    if (load_sample(inst_sel, path, err, sizeof err) == 0) {
        snprintf(status, sizeof status, "Loaded into slot %d: %s", inst_sel + 1, ins[inst_sel].name);
        peaks_dirty = 1;
    } else {
        snprintf(status, sizeof status, "Load failed: %s", err);
    }
}

static void do_save(void)
{
    char path[PATHLEN], msg[160], err[96];
    snprintf(msg, sizeof msg, "Save project as [%s]: ",
             proj_path[0] ? base_name(proj_path) : "cancel");
    int got = prompt_line(msg, path, sizeof path);
    if (!got) {
        if (!proj_path[0]) { status[0] = 0; return; }
        snprintf(path, sizeof path, "%s", proj_path);
    }
    ensure_ext(path, sizeof path);
    if (project_save(path, err, sizeof err) == 0) {
        snprintf(proj_path, sizeof proj_path, "%s", path);
        snprintf(status, sizeof status, "Saved %s", path);
    } else {
        snprintf(status, sizeof status, "Save failed: %s", err);
    }
}

static void do_load(void)
{
    char path[PATHLEN], err[96];
    int missing = 0;
    if (!prompt_line("Load project (empty = cancel): ", path, sizeof path)) { status[0] = 0; return; }
    ensure_ext(path, sizeof path);
    if (project_load(path, err, sizeof err, &missing) == 0) {
        peaks_dirty = 1;
        cur_row = 0;
        if (missing) snprintf(status, sizeof status, "Loaded %s (%d sample file(s) not found)", path, missing);
        else         snprintf(status, sizeof status, "Loaded %s", path);
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

/* mag: 0 = 1 unit, 1 = normal, 2 = big */
static void adjust_param(int dir, int mag)
{
    Inst *in = &ins[inst_sel];
    switch (inst_param) {
    case P_TUNE:
        in->tune += dir * (mag == 2 ? 12 : 1);
        if (in->tune < -48) in->tune = -48;
        if (in->tune > 48)  in->tune = 48;
        break;
    case P_FINE:
        in->fine += dir * (mag == 2 ? 10 : 1);
        if (in->fine < -99) in->fine = -99;
        if (in->fine > 99)  in->fine = 99;
        break;
    case P_VOL:
        in->vol += dir * (mag == 2 ? 16 : 1);
        if (in->vol < 0)   in->vol = 0;
        if (in->vol > 255) in->vol = 255;
        break;
    case P_LOOP:
        in->loop = (in->loop + dir + 3) % 3;
        break;
    case P_ATT: case P_DEC: case P_REL: {
        int *t = inst_param == P_ATT ? &in->att_ms : inst_param == P_DEC ? &in->dec_ms : &in->rel_ms;
        *t += dir * (mag == 0 ? 1 : mag == 1 ? 10 : 100);
        if (*t < 0) *t = 0;
        if (*t > MAX_ENV_MS) *t = MAX_ENV_MS;
        break;
    }
    case P_SUS:
        in->sus += dir * (mag == 2 ? 16 : 1);
        if (in->sus < 0)   in->sus = 0;
        if (in->sus > 255) in->sus = 255;
        break;
    case P_LSTART:
    case P_LEND: {
        if (in->len < 2) break;
        int64_t unit = mag == 0 ? 1 : mag == 1 ? in->len / 1000 : in->len / 50;
        if (unit < 1) unit = 1;
        int64_t d = dir * unit;
        if (inst_param == P_LSTART) {
            int64_t v = (int64_t)in->loop_start + d;
            int64_t mx = (int64_t)in->loop_end - 2;
            if (v > mx) v = mx;
            if (v < 0)  v = 0;
            in->loop_start = (uint32_t)v;
        } else {
            int64_t v = (int64_t)in->loop_end + d;
            int64_t lo = (int64_t)in->loop_start + 2;
            if (v < lo) v = lo;
            if (v > (int64_t)in->len) v = in->len;
            in->loop_end = (uint32_t)v;
        }
        break;
    }
    }
}

static void inst_key(int k)
{
    switch (k) {
    case KEY_UP:    inst_sel = (inst_sel + NINST - 1) % NINST; cur_inst = inst_sel; peaks_dirty = 1; return;
    case KEY_DOWN:  inst_sel = (inst_sel + 1) % NINST;         cur_inst = inst_sel; peaks_dirty = 1; return;
    case '\t':      inst_param = (inst_param + 1) % NPARAM; return;
    case KEY_BTAB:  inst_param = (inst_param + NPARAM - 1) % NPARAM; return;
    case KEY_LEFT:  adjust_param(-1, 1); return;
    case KEY_RIGHT: adjust_param(+1, 1); return;
    case ',':       adjust_param(-1, 0); return;
    case '.':       adjust_param(+1, 0); return;
    case KEY_PPAGE: adjust_param(+1, 2); return;
    case KEY_NPAGE: adjust_param(-1, 2); return;
    case '1':       audition(inst_sel, NOTE_OFF); return;
    case 'l': case 'L': case '\n': case KEY_ENTER: prompt_load_sample(); return;
    case KEY_DC:
        swap_sample(inst_sel, NULL, 0, 0, "");
        ins[inst_sel].path[0] = 0;
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
        if (!edit_mode) { snprintf(status, sizeof status, "Edit mode is off - press F7 to edit"); return; }
        if (cur_field == 0) { ce->note = 0; ce->inst = 0; ce->vol = -1; }
        else if (cur_field == 1) ce->inst = 0;
        else ce->vol = -1;
        move_row(step);
        return;
    }

    if (!edit_mode) {                 /* view mode: audition only, never touch the pattern */
        int n = key_to_note(k, octave);
        if (n)            audition(cur_inst, n);
        else if (k == '1') audition(cur_inst, NOTE_OFF);
        else if (cur_field >= 1 && (hexval(k) >= 0))
            snprintf(status, sizeof status, "Edit mode is off - press F7 to edit");
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
    case KEY_F(5): do_save(); break;
    case KEY_F(6): do_load(); break;
    case KEY_F(7):
        edit_mode = !edit_mode;
        snprintf(status, sizeof status, edit_mode ? "Edit mode ON" : "Edit mode OFF (view / audition only)");
        break;
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
    for (int i = 0; i < NINST; i++) { ins[i].vol = 255; ins[i].sus = 255; }
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

    int slot = 0;
    for (int i = 1; i < argc; i++) {
        char err[96];
        size_t l = strlen(argv[i]);
        if (l > 4 && strcmp(argv[i] + l - 4, ".xtp") == 0) {
            int missing = 0;
            if (project_load(argv[i], err, sizeof err, &missing) != 0)
                fprintf(stderr, "xtrack: %s: %s\n", argv[i], err);
            else if (missing)
                fprintf(stderr, "xtrack: %s: %d sample file(s) not found\n", argv[i], missing);
        } else if (slot < NINST) {
            if (load_sample(slot, argv[i], err, sizeof err) != 0)
                fprintf(stderr, "xtrack: %s: %s\n", argv[i], err);
            slot++;
        }
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
        init_pair(5, COLOR_WHITE, COLOR_RED);     /* edit-mode indicator */
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
