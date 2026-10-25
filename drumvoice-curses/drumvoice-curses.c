/*
 * drumvoice-curses - tiny analog-style drum voice synth, ncurses front end
 *   JACK MIDI in -> 4 voices (BD, SD, closed hat, open hat) -> JACK stereo out
 *   Terminal UI (sized for a 1024x768 Linux console, 128x48 chars) with
 *   tune / decay / bend / tone / level per voice.
 *
 * Default MIDI notes (GM): BD 36, SD 38, CH 42, OH 46
 * Override: ./drumvoice-curses -b 36 -s 38 -c 42 -o 46 -n clientname
 *
 * Incoming note-ons and CCs are shown in the MIDI monitor, so you can find the
 * numbers your controller sends. Then map them:
 *   -C t,d,b,x,l   five CC numbers -> TUNE, DECAY, BEND, tone(CLICK/NOISE/RESO), LEVEL
 *   -S a,b,c,d     four note numbers -> select BD, SD, CH, OH as the voice the
 *                  five CCs control (use -1 to leave a slot unmapped)
 * e.g. ./drumvoice-curses -C 74,71,76,77,7 -S 40,41,43,44
 */

#define _GNU_SOURCE
#include <ncurses.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PI      3.14159265358979f
#define TWO_PI  6.28318530717959f

#define NV 4            /* voices */
#define NP 5            /* params per voice */

enum { P_TUNE, P_DECAY, P_BEND, P_TONE, P_LEVEL };
enum { T_BD, T_SD, T_HH };

/* ------------------------------------------------------------------ */
/* Voice definitions: ranges and defaults (all params stored 0..1)     */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *pad, *tone_name;
    int   type;
    float tune_lo, tune_hi;   /* Hz, exponential map                  */
    float dec_lo, dec_hi;     /* seconds (to -60 dB), exponential map */
    float bend_max;           /* start pitch = tune * (1 + bend)      */
    float ptau;               /* pitch envelope time constant (s)     */
    float def[NP];            /* tune, decay, bend, tone, level       */
} VoiceDef;

static const VoiceDef defs[NV] = {
    { "BD", "CLICK", T_BD,   30.f,   120.f, 0.05f, 1.2f, 6.f, 0.030f, {0.35f, 0.45f, 0.45f, 0.25f, 0.80f} },
    { "SD", "NOISE", T_SD,  120.f,   400.f, 0.04f, 0.6f, 2.f, 0.015f, {0.40f, 0.50f, 0.30f, 0.60f, 0.80f} },
    { "CH", "RESO",  T_HH, 3000.f, 14000.f, 0.01f, 0.25f, 1.5f, 0.020f, {0.70f, 0.30f, 0.00f, 0.30f, 0.70f} },
    { "OH", "RESO",  T_HH, 3000.f, 14000.f, 0.10f, 2.0f, 1.5f, 0.020f, {0.70f, 0.50f, 0.00f, 0.30f, 0.70f} },
};
static const char *pname[NP] = { "TUNE", "DECAY", "BEND", NULL, "LEVEL" };

/* ------------------------------------------------------------------ */
/* Shared state (UI thread writes, audio thread reads)                 */
/* ------------------------------------------------------------------ */
static volatile float P[NV][NP];
static atomic_int trig[NV];      /* UI -> audio: velocity (1..127) */
static atomic_int hit[NV];       /* audio -> UI: voice was triggered */
static atomic_int quit_flag;
static int midi_note[NV] = { 30, 31, 32, 33 };
static int cc_map[NP]    = { 7,12,13,14,15 };   /* CC number per param, -1 = unmapped */
static int sel_note[NV]  = { 34,35,36,37 };       /* note that selects each voice       */
static atomic_int sel_voice;                          /* voice the CC knobs control         */

/* lock-free single-producer/single-consumer queue: audio thread -> UI thread
 * (so printing never happens in the realtime callback) */
typedef struct { uint8_t st, d1, d2; } MidiMsg;
#define RING_N 256
static MidiMsg ring[RING_N];
static atomic_uint rhead, rtail;

static void ring_push(uint8_t st, uint8_t d1, uint8_t d2)
{
    unsigned h = atomic_load_explicit(&rhead, memory_order_relaxed);
    unsigned t = atomic_load_explicit(&rtail, memory_order_acquire);
    if (h - t >= RING_N) return;
    ring[h % RING_N] = (MidiMsg){ st, d1, d2 };
    atomic_store_explicit(&rhead, h + 1, memory_order_release);
}

static int ring_pop(MidiMsg *m)
{
    unsigned t = atomic_load_explicit(&rtail, memory_order_relaxed);
    unsigned h = atomic_load_explicit(&rhead, memory_order_acquire);
    if (t == h) return 0;
    *m = ring[t % RING_N];
    atomic_store_explicit(&rtail, t + 1, memory_order_release);
    return 1;
}

static float expmap(float n, float lo, float hi) { return lo * powf(hi / lo, n); }
static float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

/* ------------------------------------------------------------------ */
/* DSP                                                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    int      active, choke;
    float    phase, phase2;
    float    env, env2, penv;   /* env: main amp, env2: click / osc amp */
    float    vel;
    float    ic1, ic2;          /* SVF state */
    uint32_t rng;
} Voice;

typedef struct {                /* per-block mapped parameters */
    float tune, bend, tone, level;
    float coef, coef2, pcoef, chokecoef;
    float g_hp;                 /* snare noise highpass */
    float k, sqrtk;             /* hat resonance */
} MP;

static Voice vo[NV];
static MP    mp[NV];
static float g_sr = 48000.f;

static float dec_coef(float t60, float sr) { return expf(-6.9078f / (t60 * sr)); }
static float tc_coef(float tau, float sr)  { return expf(-1.f / (tau * sr)); }

static inline float noise(Voice *v)
{
    v->rng ^= v->rng << 13;
    v->rng ^= v->rng >> 17;
    v->rng ^= v->rng << 5;
    return (float)(int32_t)v->rng * (1.0f / 2147483648.0f);
}

/* Zavalishin TPT state-variable filter: stable at any cutoff */
static inline void svf(Voice *v, float in, float g, float k, float *bp, float *hp)
{
    float a1 = 1.f / (1.f + g * (g + k));
    float a2 = g * a1, a3 = g * a2;
    float v3 = in - v->ic2;
    float v1 = a1 * v->ic1 + a2 * v3;
    float v2 = v->ic2 + a2 * v->ic1 + a3 * v3;
    v->ic1 = 2.f * v1 - v->ic1;
    v->ic2 = 2.f * v2 - v->ic2;
    *bp = v1;
    *hp = in - k * v1 - v2;
}

static void voice_trigger(int i, float vel)
{
    Voice *v = &vo[i];
    v->active = 1;
    v->choke  = 0;
    v->env = v->env2 = v->penv = 1.f;
    v->phase = v->phase2 = 0.f;
    v->vel = vel;
    atomic_store(&hit[i], 1);
    if (i == 2 && vo[3].active)     /* closed hat chokes open hat */
        vo[3].choke = 1;
}

static void compute_params(void)
{
    for (int i = 0; i < NV; i++) {
        const VoiceDef *d = &defs[i];
        MP *m = &mp[i];
        float dec = expmap(clamp01(P[i][P_DECAY]), d->dec_lo, d->dec_hi);
        m->tune  = expmap(clamp01(P[i][P_TUNE]), d->tune_lo, d->tune_hi);
        m->bend  = clamp01(P[i][P_BEND]) * d->bend_max;
        m->tone  = clamp01(P[i][P_TONE]);
        m->level = clamp01(P[i][P_LEVEL]);
        m->level *= m->level;
        m->coef  = dec_coef(dec, g_sr);
        m->pcoef = tc_coef(d->ptau, g_sr);
        m->chokecoef = dec_coef(0.010f, g_sr);
        switch (d->type) {
        case T_BD: m->coef2 = tc_coef(0.002f, g_sr); break;          /* click */
        case T_SD: m->coef2 = dec_coef(dec * 0.45f, g_sr); break;    /* osc   */
        default:   m->coef2 = m->coef; break;
        }
        m->g_hp  = tanf(PI * 1800.f / g_sr);
        m->k     = 1.f / (0.7f + m->tone * 20.f);
        m->sqrtk = sqrtf(m->k);
    }
}

static void render_seg(float *buf, uint32_t a, uint32_t b)
{
    const float sr = g_sr;
    for (int i = 0; i < NV; i++) {
        Voice *v = &vo[i];
        if (!v->active) continue;
        const MP *m = &mp[i];
        const int type = defs[i].type;

        for (uint32_t n = a; n < b; n++) {
            float s = 0.f;
            float f = m->tune * (1.f + m->bend * v->penv);

            if (type == T_BD) {
                v->phase += f / sr;
                if (v->phase >= 1.f) v->phase -= 1.f;
                float click = noise(v) * v->env2 * m->tone;
                s = tanhf((sinf(TWO_PI * v->phase) + click * 1.5f) * (1.f + m->tone * 2.f)) * v->env;
            } else if (type == T_SD) {
                v->phase  += f / sr;          if (v->phase  >= 1.f) v->phase  -= 1.f;
                v->phase2 += f * 1.62f / sr;  if (v->phase2 >= 1.f) v->phase2 -= 1.f;
                float osc = 0.65f * sinf(TWO_PI * v->phase) + 0.35f * sinf(TWO_PI * v->phase2);
                float bp, hp;
                svf(v, noise(v), m->g_hp, 1.4f, &bp, &hp);
                float go = fminf(1.f, 2.f * (1.f - m->tone));
                float gn = fminf(1.f, 2.f * m->tone);
                s = go * osc * v->env2 + gn * hp * 1.2f * v->env;
            } else { /* T_HH: tuned noise */
                float cf = fminf(f, 0.45f * sr);
                float g  = tanf(PI * cf / sr);
                float bp, hp;
                svf(v, noise(v), g, m->k, &bp, &hp);
                s = bp * m->sqrtk * 2.f * v->env;
            }

            buf[n] += s * v->vel * m->level;

            float c1 = v->choke ? m->chokecoef : m->coef;
            float c2 = v->choke ? m->chokecoef : m->coef2;
            v->env  *= c1;
            v->env2 *= c2;
            v->penv *= m->pcoef;
        }
        if (v->env < 1e-4f && v->env2 < 1e-4f) v->active = 0;
    }
}

/* ------------------------------------------------------------------ */
/* JACK                                                                */
/* ------------------------------------------------------------------ */
static jack_client_t *client;
static jack_port_t   *midi_in, *out_l, *out_r;

static int process(jack_nframes_t nf, void *arg)
{
    (void)arg;
    float *ol = jack_port_get_buffer(out_l, nf);
    float *or_ = jack_port_get_buffer(out_r, nf);
    void  *mb = jack_port_get_buffer(midi_in, nf);

    memset(ol, 0, nf * sizeof(float));

    for (int i = 0; i < NV; i++) {
        int t = atomic_exchange(&trig[i], 0);
        if (t) voice_trigger(i, t / 127.f);
    }
    compute_params();

    uint32_t pos = 0;
    uint32_t ne = jack_midi_get_event_count(mb);
    for (uint32_t e = 0; e < ne; e++) {
        jack_midi_event_t ev;
        if (jack_midi_event_get(&ev, mb, e) != 0) continue;
        uint32_t t = ev.time < nf ? ev.time : nf;
        if (t < pos) t = pos;
        render_seg(ol, pos, t);
        pos = t;
        if (ev.size >= 3 && ev.buffer[0] >= 0x80 && ev.buffer[0] < 0xF0) {
            uint8_t st = ev.buffer[0], d1 = ev.buffer[1], d2 = ev.buffer[2];
            if ((st & 0xF0) == 0x90 && d2 > 0) {
                ring_push(st, d1, d2);
                for (int i = 0; i < NV; i++) {
                    if (d1 == midi_note[i]) voice_trigger(i, d2 / 127.f);
                    if (d1 == sel_note[i])  atomic_store(&sel_voice, i);
                }
            } else if ((st & 0xF0) == 0xB0) {
                ring_push(st, d1, d2);
                int sv = atomic_load(&sel_voice);
                for (int k = 0; k < NP; k++)
                    if (d1 == cc_map[k]) P[sv][k] = d2 / 127.f;
            }
        }
    }
    render_seg(ol, pos, nf);

    for (jack_nframes_t n = 0; n < nf; n++) {
        ol[n] = tanhf(ol[n]);
        or_[n] = ol[n];
    }
    return 0;
}

static void on_shutdown(void *arg)
{
    (void)arg;
    atomic_store(&quit_flag, 1);
}

/* ------------------------------------------------------------------ */
/* ncurses UI                                                          */
/* ------------------------------------------------------------------ */
#define BLOCK_W 62          /* width of one voice block (2 columns need 124+ cols) */
#define BLOCK_H 8
#define BAR_W   30

static const char *ch_names[NV]   = { "BD", "SD", "CH", "OH" };
static const char *full_names[NV] = { "BASS DRUM", "SNARE", "CLOSED HAT", "OPEN HAT" };

/* scrolling MIDI monitor */
#define LOG_N 128
#define LOG_W 80
static char     logbuf[LOG_N][LOG_W];
static unsigned log_count;
static char *log_slot(void) { return logbuf[log_count++ % LOG_N]; }

static void jack_quiet(const char *msg) { (void)msg; }
static void on_signal(int sig) { (void)sig; atomic_store(&quit_flag, 1); }

static void ui_trigger(int v) { atomic_store(&trig[v], 100); }

/* parse "a,b,c" into out[0..n-1]; missing entries are left unchanged */
static void parse_list(const char *str, int *out, int n)
{
    for (int i = 0; i < n && *str; i++) {
        char *end;
        long val = strtol(str, &end, 10);
        if (end == str) break;
        out[i] = (int)val;
        str = (*end == ',') ? end + 1 : end;
    }
}

static void param_text(int v, int k, char *buf, size_t sz)
{
    const VoiceDef *d = &defs[v];
    float n = clamp01(P[v][k]);
    switch (k) {
    case P_TUNE:  snprintf(buf, sz, "%.0f Hz", expmap(n, d->tune_lo, d->tune_hi)); break;
    case P_DECAY: snprintf(buf, sz, "%.0f ms", 1000.f * expmap(n, d->dec_lo, d->dec_hi)); break;
    case P_BEND:  snprintf(buf, sz, "x%.1f", 1.f + n * d->bend_max); break;
    default:      snprintf(buf, sz, "%.0f%%", n * 100.f); break;
    }
}

static void draw_ui(int cur_v, int cur_k, const int *flash, const char *cname)
{
    erase();

    attron(A_BOLD);
    mvprintw(0, 1, "DRUMVOICE   jack client: %s   %.0f Hz", cname, g_sr);
    attroff(A_BOLD);

    /* mapping summary */
    char l1[256];
    int len = snprintf(l1, sizeof l1, "CC map (tune decay bend tone level):");
    for (int k = 0; k < NP; k++) {
        if (cc_map[k] >= 0) len += snprintf(l1 + len, sizeof l1 - len, " %d", cc_map[k]);
        else                len += snprintf(l1 + len, sizeof l1 - len, " --");
    }
    len += snprintf(l1 + len, sizeof l1 - len, "   select notes (BD SD CH OH):");
    for (int v = 0; v < NV; v++) {
        if (sel_note[v] >= 0) len += snprintf(l1 + len, sizeof l1 - len, " %d", sel_note[v]);
        else                  len += snprintf(l1 + len, sizeof l1 - len, " --");
    }
    mvaddnstr(1, 1, l1, COLS - 2);

    int ncols = (COLS >= 2 * BLOCK_W) ? 2 : 1;
    int rows  = (NV + ncols - 1) / ncols;
    int sv    = atomic_load(&sel_voice);

    for (int v = 0; v < NV; v++) {
        int bx = 1 + (v % ncols) * BLOCK_W;
        int by = 3 + (v / ncols) * BLOCK_H;

        /* header */
        char hb[BLOCK_W + 16];
        snprintf(hb, sizeof hb, " %s  %-10s  note %-3d%s", ch_names[v], full_names[v],
                 midi_note[v], v == sv ? "   <- MIDI CC knobs" : "");
        attr_t ha = A_BOLD | COLOR_PAIR(2) | (flash[v] > 0 ? A_REVERSE : 0);
        attron(ha);
        mvprintw(by, bx, "%-*.*s", BLOCK_W - 4, BLOCK_W - 4, hb);
        attroff(ha);

        for (int k = 0; k < NP; k++) {
            int y = by + 1 + k;
            int cur = (v == cur_v && k == cur_k);
            const char *label = (k == P_TONE) ? defs[v].tone_name : pname[k];

            mvaddch(y, bx + 1, cur ? '>' : ' ');
            if (cur) attron(A_REVERSE | A_BOLD);
            mvprintw(y, bx + 3, "%-6s", label);
            if (cur) attroff(A_REVERSE | A_BOLD);

            int x0 = bx + 10;
            int fill = (int)(clamp01(P[v][k]) * BAR_W + 0.5f);
            mvaddch(y, x0, '[');
            for (int i = 0; i < BAR_W; i++) {
                if (i < fill) {
                    attron(COLOR_PAIR(1) | A_BOLD);
                    mvaddch(y, x0 + 1 + i, '#');
                    attroff(COLOR_PAIR(1) | A_BOLD);
                } else {
                    attron(COLOR_PAIR(4));
                    mvaddch(y, x0 + 1 + i, '.');
                    attroff(COLOR_PAIR(4));
                }
            }
            mvaddch(y, x0 + BAR_W + 1, ']');

            char vb[32]; param_text(v, k, vb, sizeof vb);
            if (cur) attron(A_BOLD);
            mvprintw(y, x0 + BAR_W + 3, "%s", vb);
            if (cur) attroff(A_BOLD);
        }
    }

    /* MIDI monitor */
    int ty = 3 + rows * BLOCK_H;
    attron(A_BOLD | COLOR_PAIR(3));
    mvaddnstr(ty, 1, "MIDI monitor", COLS - 2);
    attroff(A_BOLD | COLOR_PAIR(3));
    int nlines = LINES - 2 - ty;
    if (nlines > LOG_N) nlines = LOG_N;
    if (nlines > 0) {
        unsigned shown = log_count < (unsigned)nlines ? log_count : (unsigned)nlines;
        for (unsigned i = 0; i < shown; i++) {
            unsigned idx = log_count - shown + i;
            mvaddnstr(ty + 1 + (int)i, 2, logbuf[idx % LOG_N], COLS - 3);
        }
    }

    /* help line */
    mvaddnstr(LINES - 1, 1,
        "arrows/hjkl move+adjust  PgUp/PgDn coarse  [ ] fine  r reset  "
        "1-4/space trigger  s select CC voice  q quit", COLS - 2);
    refresh();
}

static void adjust(int v, int k, float d) { P[v][k] = clamp01(P[v][k] + d); }

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    const char *cname = "drumvoice";
    int opt;
    while ((opt = getopt(argc, argv, "b:s:c:o:n:C:S:h")) != -1) {
        switch (opt) {
        case 'b': midi_note[0] = atoi(optarg) & 127; break;
        case 's': midi_note[1] = atoi(optarg) & 127; break;
        case 'c': midi_note[2] = atoi(optarg) & 127; break;
        case 'o': midi_note[3] = atoi(optarg) & 127; break;
        case 'n': cname = optarg; break;
        case 'C': parse_list(optarg, cc_map, NP); break;
        case 'S': parse_list(optarg, sel_note, NV); break;
        default:
            fprintf(stderr, "usage: %s [-b bd_note] [-s sd_note] [-c closed_hat_note] "
                            "[-o open_hat_note] [-n jack_client_name]\n"
                            "          [-C tune,decay,bend,tone,level CCs] [-S bd,sd,ch,oh select notes]\n", argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }

    for (int i = 0; i < NV; i++) {
        for (int k = 0; k < NP; k++) P[i][k] = defs[i].def[k];
        vo[i].rng = 0x12345678u + (uint32_t)i * 7919u;
    }

    /* JACK */
    jack_status_t st;
    client = jack_client_open(cname, JackNoStartServer, &st);
    if (!client) {
        fprintf(stderr, "Could not connect to JACK (is the server running?)\n");
        return 1;
    }
    jack_set_error_function(jack_quiet);   /* keep JACK from scribbling over the screen */
    jack_set_info_function(jack_quiet);
    g_sr = (float)jack_get_sample_rate(client);
    midi_in = jack_port_register(client, "midi_in", JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0);
    out_l   = jack_port_register(client, "out_l", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    out_r   = jack_port_register(client, "out_r", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    if (!midi_in || !out_l || !out_r) { fprintf(stderr, "Port registration failed\n"); return 1; }
    jack_set_process_callback(client, process, NULL);
    jack_on_shutdown(client, on_shutdown, NULL);
    if (jack_activate(client)) { fprintf(stderr, "Cannot activate client\n"); return 1; }

    const char **ports = jack_get_ports(client, NULL, JACK_DEFAULT_AUDIO_TYPE,
                                        JackPortIsPhysical | JackPortIsInput);
    if (ports) {
        if (ports[0]) jack_connect(client, jack_port_name(out_l), ports[0]);
        if (ports[0] && ports[1]) jack_connect(client, jack_port_name(out_r), ports[1]);
        jack_free(ports);
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* ncurses */
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(33);
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN,  -1);   /* bar fill   */
        init_pair(2, COLOR_YELLOW, -1);   /* headers    */
        init_pair(3, COLOR_CYAN,   -1);   /* monitor    */
        init_pair(4, COLOR_BLUE,   -1);   /* empty bar  */
    }

    int cur_v = 0, cur_k = 0, prev_sel = 0;
    int flash[NV] = {0};

    while (!atomic_load(&quit_flag)) {
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': atomic_store(&quit_flag, 1); break;
        case KEY_UP: case 'k':
            if (--cur_k < 0) { cur_k = NP - 1; cur_v = (cur_v + NV - 1) % NV; }
            break;
        case KEY_DOWN: case 'j':
            if (++cur_k >= NP) { cur_k = 0; cur_v = (cur_v + 1) % NV; }
            break;
        case '\t':       cur_v = (cur_v + 1) % NV; break;
        case KEY_BTAB:   cur_v = (cur_v + NV - 1) % NV; break;
        case KEY_LEFT:  case 'h': adjust(cur_v, cur_k, -0.02f); break;
        case KEY_RIGHT: case 'l': adjust(cur_v, cur_k,  0.02f); break;
        case KEY_NPAGE: adjust(cur_v, cur_k, -0.10f); break;
        case KEY_PPAGE: adjust(cur_v, cur_k,  0.10f); break;
        case '[': adjust(cur_v, cur_k, -0.005f); break;
        case ']': adjust(cur_v, cur_k,  0.005f); break;
        case 'r': case KEY_DC: P[cur_v][cur_k] = defs[cur_v].def[cur_k]; break;
        case 's': atomic_store(&sel_voice, cur_v); break;
        case ' ': case '\n': case KEY_ENTER: ui_trigger(cur_v); break;
        case '1': case '2': case '3': case '4': ui_trigger(ch - '1'); break;
        default: break;
        }

        /* drain MIDI messages from the audio thread into the monitor */
        MidiMsg mm;
        while (ring_pop(&mm)) {
            int chn = (mm.st & 0x0F) + 1;
            char note[48] = "";
            if ((mm.st & 0xF0) == 0xB0) {
                for (int k = 0; k < NP; k++)
                    if (cc_map[k] == mm.d1) {
                        snprintf(note, sizeof note, "  -> %s %s", ch_names[atomic_load(&sel_voice)],
                                 k == P_TONE ? "TONE" : pname[k]);
                    }
                snprintf(log_slot(), LOG_W, "CC   ch=%-2d cc=%-3d val=%-3d%s", chn, mm.d1, mm.d2, note);
            } else {
                for (int i = 0; i < NV; i++) {
                    if (mm.d1 == midi_note[i])
                        snprintf(note, sizeof note, "  -> trigger %s", ch_names[i]);
                    if (mm.d1 == sel_note[i])
                        snprintf(note, sizeof note, "  -> select %s", ch_names[i]);
                }
                snprintf(log_slot(), LOG_W, "NOTE ch=%-2d note=%-3d vel=%-3d%s", chn, mm.d1, mm.d2, note);
            }
        }
        int sv = atomic_load(&sel_voice);
        if (sv != prev_sel) {
            snprintf(log_slot(), LOG_W, "--> MIDI CC knobs now control %s", ch_names[sv]);
            prev_sel = sv;
        }

        for (int i = 0; i < NV; i++) {
            if (atomic_exchange(&hit[i], 0)) flash[i] = 8;
            else if (flash[i] > 0) flash[i]--;
        }
        draw_ui(cur_v, cur_k, flash, cname);
    }

    endwin();
    jack_client_close(client);
    return 0;
}
