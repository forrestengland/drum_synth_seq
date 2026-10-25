/*
 * tb303_nc.c - TB-303 style monophonic bass synth + 16 step piano roll (ncurses)
 *
 * Build (Debian 11):
 *   sudo apt install build-essential libjack-jackd2-dev libncurses-dev jackd2
 *   gcc -O2 -Wall tb303_nc.c -o tb303_nc -ljack -lncurses -lm
 *
 * Run:  start jackd (or qjackctl) first, then ./tb303_nc   (terminal >= 76x14)
 *
 * Pattern view : arrows / hjkl move the cursor (step, note)
 *                RET or x   place note at cursor (again on same note = erase)
 *                a / s      toggle accent / slide on the cursor step
 *                d          clear step      K / J  transpose step up / down
 *                PgUp/PgDn  cursor one octave up / down
 * Params view  : TAB toggles; up/down select, left/right adjust, PgUp/PgDn coarse
 * Global       : SPACE play/stop, m INT/EXT sync, r random, c clear, +/- tempo, q quit
 *
 * External sync: connect a MIDI clock source to the "tb303:midi_in" JACK port.
 * In EXT mode the sequencer follows 0xF8 clock (24 ppqn, 6 ticks per step),
 * 0xFA start, 0xFB continue and 0xFC stop. The tempo slider is ignored.
 * ALSA hardware ports: run jackd with "-X seq" or use a2jmidid -e.
 */

#include <ncurses.h>
#include <signal.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STEPS     16
#define ROWS      24          /* two octaves */
#define BASE_MIDI 36          /* C2 */

/* ---------- shared state (UI thread <-> audio thread) ---------- */
typedef struct { int note; int gate; int accent; int slide; } Step;

static Step steps[STEPS];
static volatile int playing = 0;
static volatile int cur_step = -1;
static volatile int running = 1;

enum { P_CUTOFF, P_RESO, P_ENVMOD, P_DECAY, P_ACCENT, P_TEMPO, P_WAVE, P_VOLUME, P_COUNT };
static const char *param_name[P_COUNT] =
    { "CUTOFF", "RESO", "ENV MOD", "DECAY", "ACCENT", "TEMPO", "WAVE", "VOLUME" };
static volatile float param[P_COUNT] = { 0.35f, 0.6f, 0.6f, 0.4f, 0.7f, 0.35f, 0.0f, 0.7f };

/* ---------- JACK / synth ---------- */
static jack_client_t *client;
static jack_port_t *out_l, *out_r, *midi_in;
static volatile int ext_sync = 0;
static volatile float ext_bpm = 0;

static inline float mtof(float m) { return 440.0f * exp2f((m - 69.0f) / 12.0f); }

static inline float polyblep(float t, float dt)
{
    if (t < dt)         { t /= dt; return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt)  { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}

static int process(jack_nframes_t nframes, void *arg)
{
    (void)arg;
    float *ol = jack_port_get_buffer(out_l, nframes);
    float *or_ = jack_port_get_buffer(out_r, nframes);
    const float sr = (float)jack_get_sample_rate(client);

    /* voice state persists between callbacks */
    static float phase = 0, cur_freq = 110, tgt_freq = 110;
    static float aenv = 0, fenv = 0;
    static int gate = 0, acc = 0;
    static float gate_cnt = 0, step_cnt = 0;
    static int step = 0, was_playing = 0;
    static float b0, b1, b2, b3, b4;
    /* external clock state */
    static int tick_count = 0;
    static float since_tick = 0, tick_avg = 0;

    float cutoff = param[P_CUTOFF], reso = param[P_RESO] * 0.95f;
    float envmod = param[P_ENVMOD], accamt = param[P_ACCENT];
    float bpm = 60.0f + param[P_TEMPO] * 140.0f;
    int square = param[P_WAVE] > 0.5f;
    float vol = param[P_VOLUME];
    float int_step_len = sr * 60.0f / bpm / 4.0f;           /* 16th notes */
    float dec_t = 0.2f + param[P_DECAY] * 1.8f;             /* seconds */
    int ext = ext_sync;

    float glide_c = 1.0f - expf(-1.0f / (0.06f * sr));
    float atk_c   = 1.0f - expf(-1.0f / (0.003f * sr));
    float rel_c   = expf(-1.0f / (0.012f * sr));

    void *mbuf = jack_port_get_buffer(midi_in, nframes);
    jack_nframes_t nev = jack_midi_get_event_count(mbuf), ei = 0;
    jack_midi_event_t ev;
    int have = nev > 0 && jack_midi_event_get(&ev, mbuf, 0) == 0;

    for (jack_nframes_t i = 0; i < nframes; i++) {
        int do_step = 0;

        /* local start (space bar): restart from step 1 */
        if (playing && !was_playing) { step = STEPS - 1; step_cnt = 0; tick_count = 0; }
        was_playing = playing;

        /* MIDI realtime messages, handled at their exact frame offset */
        while (have && ev.time <= i) {
            if (ev.size >= 1 && ext) {
                switch (ev.buffer[0]) {
                case 0xF8:  /* clock */
                    if (since_tick > 0.0f) {
                        tick_avg = tick_avg > 0.0f ? tick_avg * 0.9f + since_tick * 0.1f
                                                   : since_tick;
                        ext_bpm = 60.0f * sr / (tick_avg * 24.0f);
                    }
                    since_tick = 0.0f;
                    if (playing) {
                        if (tick_count % 6 == 0) do_step = 1;
                        tick_count++;
                    }
                    break;
                case 0xFA:  /* start: first clock afterwards plays step 1 */
                    step = STEPS - 1; step_cnt = 0; tick_count = 0;
                    playing = 1; was_playing = 1;
                    break;
                case 0xFB:  /* continue */
                    playing = 1; was_playing = 1;
                    break;
                case 0xFC:  /* stop */
                    playing = 0;
                    break;
                }
            }
            ei++;
            have = ei < nev && jack_midi_event_get(&ev, mbuf, ei) == 0;
        }
        since_tick += 1.0f;

        float step_len = (ext && tick_avg > 0.0f) ? tick_avg * 6.0f : int_step_len;

        if (playing) {
            if (!ext) {
                if (step_cnt <= 0) { do_step = 1; step_cnt += int_step_len; }
                step_cnt -= 1.0f;
            }
            if (do_step) {
                step = (step + 1) % STEPS;
                cur_step = step;
                Step *s = &steps[step];
                if (s->gate) {
                    int legato = s->slide && gate;
                    tgt_freq = mtof((float)(BASE_MIDI + s->note));
                    if (!s->slide) cur_freq = tgt_freq;
                    acc = s->accent;
                    if (!legato) fenv = 1.0f;
                    gate = 1;
                    Step *nx = &steps[(step + 1) % STEPS];
                    gate_cnt = (nx->gate && nx->slide) ? step_len * 2.0f : step_len * 0.6f;
                }
            }
            if (gate && --gate_cnt <= 0) gate = 0;
        } else {
            gate = 0;
            cur_step = -1;
        }

        /* oscillator */
        cur_freq += (tgt_freq - cur_freq) * glide_c;
        float dt = cur_freq / sr;
        phase += dt;
        if (phase >= 1.0f) phase -= 1.0f;
        float osc;
        if (square) {
            osc = phase < 0.5f ? 1.0f : -1.0f;
            osc += polyblep(phase, dt);
            float p2 = phase + 0.5f; if (p2 >= 1.0f) p2 -= 1.0f;
            osc -= polyblep(p2, dt);
        } else {
            osc = 2.0f * phase - 1.0f;
            osc -= polyblep(phase, dt);
        }

        /* envelopes */
        if (gate) aenv += (1.0f - aenv) * atk_c; else aenv *= rel_c;
        float t = acc ? 0.12f : dec_t;
        fenv *= expf(-1.0f / (t * sr));

        /* filter: 4-pole resonant ladder (Stilson/Smith style) */
        float hz = 50.0f * exp2f(cutoff * 6.0f);
        hz *= exp2f(envmod * fenv * (3.5f + (acc ? accamt * 2.0f : 0.0f)));
        float fc = 2.0f * hz / sr;
        if (fc > 0.95f) fc = 0.95f;
        float q = 1.0f - fc;
        float p = fc + 0.8f * fc * q;
        float f = p + p - 1.0f;
        q = (reso + (acc ? accamt * 0.1f : 0.0f)) * (1.0f + 0.5f * q * (1.0f - q + 5.6f * q * q));

        float in = osc * 0.5f - q * b4;
        float t1 = b1; b1 = (in + b0) * p - b1 * f;
        float t2 = b2; b2 = (b1 + t1) * p - b2 * f;
        t1 = b3;       b3 = (b2 + t2) * p - b3 * f;
        b4 = (b3 + t1) * p - b4 * f;
        b4 -= b4 * b4 * b4 * 0.166667f;
        b0 = in;

        float gain = 1.0f + (acc ? accamt * 0.8f : 0.0f);
        float out = tanhf(b4 * aenv * gain * 3.0f) * vol;
        ol[i] = or_[i] = out;
    }
    return 0;
}

static void on_jack_shutdown(void *arg) { (void)arg; running = 0; }

/* ---------- pattern helpers ---------- */
static void init_pattern(void)
{
    static const int n[STEPS] = { 0, 0, 12, 0, 3, 0, 15, 7, 0, 0, 12, 10, 0, 3, 7, 5 };
    static const int g[STEPS] = { 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0 };
    static const int a[STEPS] = { 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0 };
    static const int s[STEPS] = { 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 };
    for (int i = 0; i < STEPS; i++)
        steps[i] = (Step){ n[i], g[i], a[i], s[i] };
}

static void randomize_pattern(void)
{
    static const int scale[] = { 0, 0, 3, 5, 7, 10, 12, 12, 15 };
    for (int i = 0; i < STEPS; i++) {
        steps[i].note = scale[rand() % 9];
        steps[i].gate = (rand() % 100) < 75;
        steps[i].accent = (rand() % 100) < 25;
        steps[i].slide = (rand() % 100) < 20;
    }
}

/* ---------- helpers ---------- */
static const char *note_names[12] =
    { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

static int is_black(int pc) { return pc == 1 || pc == 3 || pc == 6 || pc == 8 || pc == 10; }

static void note_name(int n, char *buf, size_t len)
{
    int m = BASE_MIDI + n;
    snprintf(buf, len, "%s%d", note_names[m % 12], m / 12 - 1);
}

static void quiet(const char *msg) { (void)msg; }
static void on_signal(int sig) { (void)sig; running = 0; }

enum { C_NOTE = 1, C_ACCENT, C_SLIDE, C_TITLE, C_PLAY, C_EXT };
static int has_col;
static int a_note(void)   { return has_col ? COLOR_PAIR(C_NOTE)   : A_REVERSE; }
static int a_accent(void) { return has_col ? COLOR_PAIR(C_ACCENT) : A_REVERSE | A_BOLD; }
static int a_slide(void)  { return has_col ? COLOR_PAIR(C_SLIDE)  : A_REVERSE; }
static int a_title(void)  { return has_col ? COLOR_PAIR(C_TITLE)  : 0; }
static int a_play(void)   { return has_col ? COLOR_PAIR(C_PLAY)   : A_BOLD; }
static int a_ext(void)    { return has_col ? COLOR_PAIR(C_EXT)    : A_BOLD; }

static void put(int y, int x, int attr, const char *s)
{
    attron(attr);
    mvaddstr(y, x, s);
    attroff(attr);
}

static void adjust(int i, float d)
{
    float v = param[i] + d;
    param[i] = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
}

#define RXC 5      /* x of first step column */
#define PXC 54     /* x of parameter panel */

static void draw_lane(int y, const char *label, int which, int cx, int focus)
{
    int at = which ? a_slide() : a_accent();
    put(y, 0, A_BOLD, label);
    for (int c = 0; c < STEPS; c++) {
        int x = RXC + c * 3;
        int on = which ? steps[c].slide : steps[c].accent;
        int iscur = !focus && c == cx;
        if (iscur)    put(y, x, (on ? at : 0) | A_BOLD, "[]");
        else if (on)  put(y, x, at | A_BOLD, which ? "S " : "A ");
        else          put(y, x, A_DIM, ". ");
        put(y, x + 2, A_DIM, (c % 4 == 3 && c < STEPS - 1) ? "|" : " ");
    }
}

static void draw_params(int focus, int psel)
{
    put(1, PXC, a_title() | A_BOLD, "PARAMS");
    for (int i = 0; i < P_COUNT; i++) {
        int y = 2 + i;
        char val[16], bar[16];
        if (i == P_TEMPO)
            snprintf(val, sizeof val, "%d", ext_sync ? (int)(ext_bpm + 0.5f)
                                                      : (int)(60 + param[i] * 140 + 0.5f));
        else if (i == P_WAVE)
            snprintf(val, sizeof val, "%s", param[i] > 0.5f ? "SQR" : "SAW");
        else
            snprintf(val, sizeof val, "%d", (int)(param[i] * 100 + 0.5f));

        int filled = (int)(param[i] * 8 + 0.5f);
        for (int k = 0; k < 8; k++) bar[k] = k < filled ? '#' : '-';
        bar[8] = 0;

        char name[16];
        snprintf(name, sizeof name, "%-7s", param_name[i]);
        put(y, PXC, (focus && i == psel) ? A_REVERSE : 0, name);
        put(y, PXC + 8, (focus && i == psel) ? A_BOLD : 0, "[");
        put(y, PXC + 9, (focus && i == psel) ? A_BOLD : 0, bar);
        put(y, PXC + 17, (focus && i == psel) ? A_BOLD : 0, "]");
        char vb[16];
        snprintf(vb, sizeof vb, "%4s", val);
        put(y, PXC + 18, (i == P_TEMPO && ext_sync) ? a_ext() : 0, vb);
    }
}

int main(void)
{
    srand(1303);
    init_pattern();

    /* JACK */
    jack_status_t st;
    client = jack_client_open("tb303", JackNoStartServer, &st);
    if (!client) { fprintf(stderr, "Cannot connect to JACK. Is jackd running?\n"); return 1; }
    out_l = jack_port_register(client, "out_l", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    out_r = jack_port_register(client, "out_r", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
    midi_in = jack_port_register(client, "midi_in", JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0);
    jack_set_process_callback(client, process, NULL);
    jack_on_shutdown(client, on_jack_shutdown, NULL);
    if (jack_activate(client)) { fprintf(stderr, "Cannot activate JACK client\n"); return 1; }

    const char **ports = jack_get_ports(client, NULL, NULL, JackPortIsPhysical | JackPortIsInput);
    if (ports) {
        if (ports[0]) jack_connect(client, jack_port_name(out_l), ports[0]);
        if (ports[0] && ports[1]) jack_connect(client, jack_port_name(out_r), ports[1]);
        jack_free(ports);
    }
    /* keep JACK from scribbling over the curses screen */
    jack_set_error_function(quiet);
    jack_set_info_function(quiet);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* curses */
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(40);                       /* ~25 fps redraw */
    has_col = has_colors();
    if (has_col) {
        start_color();
        use_default_colors();
        init_pair(C_NOTE,   COLOR_BLACK, COLOR_YELLOW);
        init_pair(C_ACCENT, COLOR_WHITE, COLOR_RED);
        init_pair(C_SLIDE,  COLOR_BLACK, COLOR_CYAN);
        init_pair(C_TITLE,  COLOR_YELLOW, -1);
        init_pair(C_PLAY,   COLOR_GREEN, -1);
        init_pair(C_EXT,    COLOR_CYAN, -1);
    }

    int cx = 0, cn = 12, focus = 0, psel = 0, view_hi = ROWS - 1;

    while (running) {
        int ch = getch();

        /* ---- input ---- */
        switch (ch) {
        case 'q': case 'Q': running = 0; break;
        case ' ': playing = !playing; break;
        case 'm': case 'M': ext_sync = !ext_sync; break;
        case 'r': case 'R': randomize_pattern(); break;
        case 'c': case 'C': memset(steps, 0, sizeof steps); break;
        case '\t': focus = !focus; break;
        case '+': case '=': adjust(P_TEMPO, 1.0f / 140.0f); break;
        case '-': case '_': adjust(P_TEMPO, -1.0f / 140.0f); break;
        default:
            if (focus) {
                switch (ch) {
                case KEY_UP:   case 'k': psel = (psel + P_COUNT - 1) % P_COUNT; break;
                case KEY_DOWN: case 'j': psel = (psel + 1) % P_COUNT; break;
                case KEY_LEFT: case 'h':
                    if (psel == P_WAVE) param[P_WAVE] = 0.0f; else adjust(psel, -0.02f);
                    break;
                case KEY_RIGHT: case 'l':
                    if (psel == P_WAVE) param[P_WAVE] = 1.0f; else adjust(psel, 0.02f);
                    break;
                case KEY_PPAGE: adjust(psel, 0.1f); break;
                case KEY_NPAGE: adjust(psel, -0.1f); break;
                }
            } else {
                switch (ch) {
                case KEY_LEFT:  case 'h': cx = (cx + STEPS - 1) % STEPS; break;
                case KEY_RIGHT: case 'l': cx = (cx + 1) % STEPS; break;
                case KEY_UP:    case 'k': if (cn < ROWS - 1) cn++; break;
                case KEY_DOWN:  case 'j': if (cn > 0) cn--; break;
                case KEY_PPAGE: cn = cn + 12 > ROWS - 1 ? ROWS - 1 : cn + 12; break;
                case KEY_NPAGE: cn = cn - 12 < 0 ? 0 : cn - 12; break;
                case KEY_HOME:  cx = 0; break;
                case KEY_END:   cx = STEPS - 1; break;
                case '\n': case '\r': case KEY_ENTER: case 'x': case 'X':
                    if (steps[cx].gate && steps[cx].note == cn) steps[cx].gate = 0;
                    else { steps[cx].note = cn; steps[cx].gate = 1; }
                    break;
                case 'a': case 'A': steps[cx].accent ^= 1; break;
                case 's': case 'S': steps[cx].slide ^= 1; break;
                case 'd': case KEY_DC: case KEY_BACKSPACE: steps[cx].gate = 0; break;
                case 'K': if (steps[cx].note < ROWS - 1) steps[cx].note++; break;
                case 'J': if (steps[cx].note > 0) steps[cx].note--; break;
                }
            }
            break;
        }

        /* ---- draw ---- */
        int H, W;
        getmaxyx(stdscr, H, W);
        erase();
        if (W < 76 || H < 14) {
            mvprintw(0, 0, "Terminal too small: need 76x14 (have %dx%d)", W, H);
            refresh();
            continue;
        }

        int vis = H - 7;
        if (vis > ROWS) vis = ROWS;
        if (cn > view_hi) view_hi = cn;
        if (cn < view_hi - vis + 1) view_hi = cn + vis - 1;
        if (view_hi > ROWS - 1) view_hi = ROWS - 1;
        if (view_hi < vis - 1) view_hi = vis - 1;

        int cs = cur_step;

        /* header */
        put(0, 0, a_title() | A_BOLD, "TB-303");
        put(0, 8, playing ? a_play() | A_BOLD : A_DIM, playing ? "> PLAY " : "[] STOP");
        char sb[48];
        if (ext_sync) snprintf(sb, sizeof sb, "SYNC EXT %3d bpm", (int)(ext_bpm + 0.5f));
        else          snprintf(sb, sizeof sb, "SYNC INT %3d bpm", (int)(60 + param[P_TEMPO] * 140 + 0.5f));
        put(0, 18, ext_sync ? a_ext() : 0, sb);
        put(0, PXC, A_DIM, focus ? "[params]" : "[pattern]");

        /* step numbers */
        for (int c = 0; c < STEPS; c++) {
            char nb[8];
            snprintf(nb, sizeof nb, "%2d", c + 1);
            int at = (c % 4 == 0) ? A_BOLD : A_DIM;
            if (!focus && c == cx) at = A_REVERSE;
            if (c == cs) at = a_play() | A_BOLD | A_REVERSE;
            put(1, RXC + c * 3, at, nb);
        }

        /* piano roll */
        for (int r = 0; r < vis; r++) {
            int n = view_hi - r, y = 2 + r;
            int pc = (BASE_MIDI + n) % 12, blk = is_black(pc);
            char lb[8], lab[16];
            note_name(n, lb, sizeof lb);
            snprintf(lab, sizeof lab, "%-4s", lb);
            put(y, 0, (!focus && n == cn) ? A_REVERSE : (pc == 0 ? A_BOLD : A_DIM), lab);
            if (r == 0 && view_hi < ROWS - 1) put(y, 4, A_BOLD, "^");
            if (r == vis - 1 && n > 0)        put(y, 4, A_BOLD, "v");

            for (int c = 0; c < STEPS; c++) {
                int x = RXC + c * 3;
                Step *s = &steps[c];
                int isnote = s->gate && s->note == n;
                int iscur = !focus && c == cx && n == cn;
                const char *gap = (c % 4 == 3 && c < STEPS - 1) ? "|" : " ";
                if (isnote) {
                    int at = s->accent ? a_accent() : a_note();
                    put(y, x, at | A_BOLD, iscur ? "[]" : "  ");
                    if (s->slide) put(y, x + 2, at, " ");   /* tie into next step */
                    else          put(y, x + 2, A_DIM, gap);
                } else {
                    if (iscur)         put(y, x, A_BOLD, "[]");
                    else if (c == cs)  put(y, x, a_play(), ": ");
                    else if (!blk)     put(y, x, A_DIM, ". ");
                    else               put(y, x, 0, "  ");
                    put(y, x + 2, A_DIM, gap);
                }
            }
        }

        draw_lane(2 + vis, "ACC", 0, cx, focus);
        draw_lane(3 + vis, "SLD", 1, cx, focus);

        draw_params(focus, psel);

        /* cursor info */
        if (12 < H - 3) {
            Step *s = &steps[cx];
            char nm[8], info[64];
            note_name(s->note, nm, sizeof nm);
            snprintf(info, sizeof info, "STEP %2d: %s%s%s", cx + 1,
                     s->gate ? nm : "rest", s->accent ? " ACC" : "", s->slide ? " SLD" : "");
            put(12, PXC, 0, info);
            if (ext_sync) put(13, PXC, a_ext(), "MIDI CLK: tb303:midi_in");
        }

        put(H - 2, 0, A_DIM,
            "arrows/hjkl move  RET note  a acc  s slide  d del  K/J transpose step  PgUp/Dn octave");
        put(H - 1, 0, A_DIM,
            "SPACE play  m sync  r rand  c clear  TAB params  +/- tempo  q quit");

        refresh();
    }

    endwin();
    jack_client_close(client);
    return 0;
}
