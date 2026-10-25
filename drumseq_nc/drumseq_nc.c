/*
 * drumseq_nc.c - 4 channel / 16 step drum sequencer, ncurses UI + JACK MIDI
 *                Lightweight console version (tested layout: 128x48 and 80x24)
 *
 * Build (Debian 11: apt install libncurses-dev libjack-jackd2-dev gcc):
 *   gcc -O2 -Wall -o drumseq_nc drumseq_nc.c -lncurses -ljack
 *
 * Run:
 *   ./drumseq_nc [target_jack_midi_port] [midi_channel 1-16] [clock_target_port]
 *   e.g. ./drumseq_nc drumsynth:midi_in 1 otherapp:midi_in
 *
 * Ports: drumseq:midi_out (notes), drumseq:clock_out (MIDI clock + start/stop)
 *
 * Keys:
 *   arrows / h j k l   move cursor
 *   x / Enter          toggle step (normal velocity)
 *   a                  toggle accent step
 *   e                  set last step of the channel at the cursor (loop length)
 *   E                  reset channel loop length to 16
 *   m  or  1-4         mute channel (m = cursor row)
 *   c / C              clear channel row / clear everything
 *   Space              play / stop
 *   + / -              BPM +/- 1        > / <   BPM +/- 10
 *   s / S              swing +/- 1
 *   v / V              accent amount +/- 1   (normal vel = 127 - amount)
 *   q                  quit
 * Mouse (xterm, or console with gpm): left toggle, right accent,
 *   middle = last step, click label = mute, wheel = BPM.
 */

#include <ncurses.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* Config                                                             */
/* ------------------------------------------------------------------ */
#define NUM_CH     4
#define NUM_STEPS  16
static const unsigned char CH_NOTE[NUM_CH] = { 30, 31, 32, 33 };
#define VEL_ACCENT 127          /* accent steps always send this */
/* normal steps send VEL_ACCENT - accent_amt (set from the UI) */
#define GATE_SECS  0.020      /* time before note-off is sent */

/* ------------------------------------------------------------------ */
/* Shared state (UI thread <-> JACK thread)                           */
/* ------------------------------------------------------------------ */
static volatile uint8_t pattern[NUM_CH][NUM_STEPS];  /* 0 off, 1 normal, 2 accent */
static _Atomic int bpm       = 120;
static _Atomic int swing_pct = 0;        /* 0..50 */
static _Atomic int accent_amt = 27;      /* 0..100, normal vel = 127 - this */
static _Atomic int playing   = 0;
static _Atomic int cur_step[NUM_CH];     /* per-channel playhead, -1 = stopped */
static _Atomic int ch_len[NUM_CH];       /* per-channel loop length 1..16 */
static _Atomic int muted[NUM_CH];
static int midi_channel = 0;             /* 0-based */

/* ------------------------------------------------------------------ */
/* JACK                                                               */
/* ------------------------------------------------------------------ */
static jack_client_t *client;
static jack_port_t   *out_port;
static jack_port_t   *clock_port;   /* MIDI clock / start / stop, 24 PPQN */

static int process(jack_nframes_t nframes, void *arg)
{
    static int    was_playing = 0;
    static int    tick = -1;                   /* global 16th parity (swing) */
    static int    ch_step[NUM_CH];
    static double next_cd = 0.0;               /* frames until next step */
    static double clk_cd  = 0.0;               /* frames until next MIDI clock */
    static long   off_cd[NUM_CH];              /* frames until note-off  */
    (void)arg;

    void *buf = jack_port_get_buffer(out_port, nframes);
    jack_midi_clear_buffer(buf);
    void *cbuf = jack_port_get_buffer(clock_port, nframes);
    jack_midi_clear_buffer(cbuf);

    const double sr = jack_get_sample_rate(client);
    const int play  = atomic_load(&playing);

    if (play != was_playing) {
        if (play) {
            tick = -1;
            for (int c = 0; c < NUM_CH; c++) ch_step[c] = -1;
            next_cd = 0.0;
            clk_cd = 0.0;
            { unsigned char m = 0xFA; jack_midi_event_write(cbuf, 0, &m, 1); } /* Start */
        } else {
            { unsigned char m = 0xFC; jack_midi_event_write(cbuf, 0, &m, 1); } /* Stop */
            for (int c = 0; c < NUM_CH; c++) {
                if (off_cd[c] > 0) {
                    unsigned char m[3] = { 0x80 | midi_channel, CH_NOTE[c], 0 };
                    jack_midi_event_write(buf, 0, m, 3);
                    off_cd[c] = 0;
                }
            }
            for (int c = 0; c < NUM_CH; c++) atomic_store(&cur_step[c], -1);
        }
        was_playing = play;
    }

    const long gate = (long)(GATE_SECS * sr);
    int normal_vel = VEL_ACCENT - atomic_load(&accent_amt);
    if (normal_vel < 1) normal_vel = 1;

    for (jack_nframes_t i = 0; i < nframes; i++) {
        /* note-offs */
        for (int c = 0; c < NUM_CH; c++) {
            if (off_cd[c] > 0 && --off_cd[c] == 0) {
                unsigned char m[3] = { 0x80 | midi_channel, CH_NOTE[c], 0 };
                jack_midi_event_write(buf, i, m, 3);
            }
        }

        if (!play) continue;

        /* MIDI clock: straight 24 PPQN (swing is not applied to the clock).
         * Starts on the same frame as step 1, so even steps line up with
         * every 12th clock pulse. */
        if (clk_cd <= 0.0) {
            unsigned char m = 0xF8;
            jack_midi_event_write(cbuf, i, &m, 1);
            clk_cd += sr * 60.0 / (double)atomic_load(&bpm) / 24.0;
        }
        clk_cd -= 1.0;

        if (next_cd <= 0.0) {
            tick = (tick + 1) & 1;

            for (int c = 0; c < NUM_CH; c++) {
                int len = atomic_load(&ch_len[c]);
                if (++ch_step[c] >= len) ch_step[c] = 0;
                atomic_store(&cur_step[c], ch_step[c]);
                uint8_t v = pattern[c][ch_step[c]];
                if (!v || atomic_load(&muted[c])) continue;

                if (off_cd[c] > 0) {   /* retrigger: close previous note */
                    unsigned char m[3] = { 0x80 | midi_channel, CH_NOTE[c], 0 };
                    jack_midi_event_write(buf, i, m, 3);
                }
                unsigned char m[3] = { 0x90 | midi_channel, CH_NOTE[c],
                                       v == 2 ? VEL_ACCENT : normal_vel };
                jack_midi_event_write(buf, i, m, 3);
                off_cd[c] = gate;
            }

            double base = sr * 60.0 / (double)atomic_load(&bpm) / 4.0; /* 16th */
            double s    = atomic_load(&swing_pct) / 100.0;
            next_cd += (tick & 1) ? base * (1.0 - s) : base * (1.0 + s);
        }
        next_cd -= 1.0;
    }
    return 0;
}


/* ------------------------------------------------------------------ */
/* UI                                                                 */
/* ------------------------------------------------------------------ */
#define LABEL_W  11
#define MIN_COLS 80
#define MIN_ROWS 20
#define CP_DIM   9

static volatile sig_atomic_t running = 1;
static int cur_ch = 0, cur_st = 0;

/* layout, recomputed on every draw so mouse hit-testing matches */
static int g_x0, g_w, g_gy, g_rowh;

static void on_signal(int sig) { (void)sig; running = 0; }
static void jack_quiet(const char *msg) { (void)msg; }  /* keep JACK off our screen */

static int cell_x(int s) { return g_x0 + LABEL_W + s * (g_w + 1) + s / 4; }

static void clamp_bpm(int v)    { atomic_store(&bpm, v < 40 ? 40 : v > 240 ? 240 : v); }
static void clamp_swing(int v)  { atomic_store(&swing_pct, v < 0 ? 0 : v > 50 ? 50 : v); }
static void clamp_accent(int v) { atomic_store(&accent_amt, v < 0 ? 0 : v > 100 ? 100 : v); }
static void toggle_play(void)   { atomic_store(&playing, !atomic_load(&playing)); }
static void toggle_mute(int c)  { atomic_store(&muted[c], !atomic_load(&muted[c])); }
static void toggle_step(int c, int s)   { pattern[c][s] = pattern[c][s] == 1 ? 0 : 1; }
static void toggle_accent(int c, int s) { pattern[c][s] = pattern[c][s] == 2 ? 0 : 2; }
static void clear_row(int c) { for (int s = 0; s < NUM_STEPS; s++) pattern[c][s] = 0; }

static void init_colors(void)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();
    init_pair(1, COLOR_RED,    -1);
    init_pair(2, COLOR_YELLOW, -1);
    init_pair(3, COLOR_GREEN,  -1);
    init_pair(4, COLOR_CYAN,   -1);
    init_pair(CP_DIM, COLOR_BLACK, -1);   /* + A_BOLD = dark grey on the console */
}

static void draw(void)
{
    erase();

    if (LINES < MIN_ROWS || COLS < MIN_COLS) {
        mvprintw(0, 0, "Terminal too small (%dx%d), need %dx%d", COLS, LINES, MIN_COLS, MIN_ROWS);
        refresh();
        return;
    }

    g_w    = (COLS >= 114) ? 5 : 3;
    g_rowh = (LINES >= 34) ? 3 : 2;
    int gridw = LABEL_W + NUM_STEPS * (g_w + 1) + 3;
    g_x0 = (COLS - gridw) / 2;
    if (g_x0 < 0) g_x0 = 0;
    g_gy = 5;

    /* header */
    attron(A_BOLD);
    mvprintw(0, g_x0, "DRUMSEQ");
    attroff(A_BOLD);
    mvprintw(0, g_x0 + 10, "JACK %uHz / %u frames",
             (unsigned)jack_get_sample_rate(client), (unsigned)jack_get_buffer_size(client));

    int play = atomic_load(&playing);
    int acc  = atomic_load(&accent_amt);
    int nv   = VEL_ACCENT - acc; if (nv < 1) nv = 1;

    int x = g_x0;
    if (play) attron(A_REVERSE | A_BOLD | (has_colors() ? COLOR_PAIR(3) : 0));
    mvprintw(2, x, play ? " PLAYING " : " STOPPED ");
    attroff(A_REVERSE | A_BOLD | COLOR_PAIR(3));
    x += 11;
    mvprintw(2, x, "BPM %-3d", atomic_load(&bpm));            x += 11;
    mvprintw(2, x, "SWING %-2d%%", atomic_load(&swing_pct));  x += 13;
    mvprintw(2, x, "ACCENT %-3d (vel %d/%d)", acc, nv, VEL_ACCENT);

    /* step numbers */
    for (int s = 0; s < NUM_STEPS; s++) {
        if (s % 4 == 0) attron(A_BOLD);
        mvprintw(g_gy - 1, cell_x(s) + (g_w - 2) / 2, "%2d", s + 1);
        attroff(A_BOLD);
    }

    /* channels */
    for (int c = 0; c < NUM_CH; c++) {
        int y     = g_gy + c * g_rowh;
        int mute  = atomic_load(&muted[c]);
        int step  = atomic_load(&cur_step[c]);
        int len   = atomic_load(&ch_len[c]);
        int cp    = has_colors() ? COLOR_PAIR(c + 1) : 0;
        int dimcp = has_colors() ? (COLOR_PAIR(CP_DIM) | A_BOLD) : A_DIM;

        /* label */
        if (mute) { attron(dimcp); mvprintw(y, g_x0, " CH%d MUTE  ", c + 1); attroff(dimcp); }
        else {
            attron(cp | A_REVERSE);
            mvprintw(y, g_x0, " CH%d N%-3d  ", c + 1, CH_NOTE[c]);
            attroff(cp | A_REVERSE);
        }
        if (c == cur_ch) mvaddch(y, g_x0 - 1, '>');

        for (int s = 0; s < NUM_STEPS; s++) {
            int v   = pattern[c][s];
            int out = s >= len;
            int cx  = cell_x(s);
            char cell[8];
            memset(cell, ' ', g_w);
            cell[g_w] = 0;
            cell[g_w / 2] = v == 2 ? 'X' : v == 1 ? 'x' : '.';

            int attr;
            if (out)            attr = dimcp;
            else if (v)         attr = cp | A_REVERSE | (v == 2 ? A_BOLD : 0) | (mute ? A_DIM : 0);
            else                attr = (s % 4 == 0) ? A_BOLD : A_NORMAL;

            if (c == cur_ch && s == cur_st) {
                cell[0] = '[';
                cell[g_w - 1] = ']';
                attr |= A_BOLD;
            }
            attron(attr);
            mvaddstr(y, cx, cell);
            attroff(attr);

            if (s == len - 1) {                          /* last-step marker */
                attron(cp | A_BOLD);
                mvaddch(y, cx + g_w, '|');
                attroff(cp | A_BOLD);
            }
            if (s == step) {                             /* playhead */
                attron(cp | A_BOLD);
                mvaddch(y + 1, cx + g_w / 2, '^');
                attroff(cp | A_BOLD);
            }
        }
    }

    /* help */
    int hy = g_gy + NUM_CH * g_rowh + 1;
    if (hy + 3 < LINES) {
        mvprintw(hy,     g_x0, "arrows/hjkl move | x,Enter toggle | a accent | e/E set/reset last step");
        mvprintw(hy + 1, g_x0, "m or 1-4 mute | c clear row | C clear all | SPACE play/stop | q quit");
        mvprintw(hy + 2, g_x0, "+/- tempo | </> tempo x10 | s/S swing | v/V accent amount");
    }
    refresh();
}

static void handle_mouse(void)
{
    MEVENT m;
    if (getmouse(&m) != OK) return;

#ifdef BUTTON4_PRESSED
    if (m.bstate & BUTTON4_PRESSED) { clamp_bpm(atomic_load(&bpm) + 1); return; }
#endif
#ifdef BUTTON5_PRESSED
    if (m.bstate & BUTTON5_PRESSED) { clamp_bpm(atomic_load(&bpm) - 1); return; }
#endif

    int rel = m.y - g_gy;
    if (rel < 0 || g_rowh == 0 || rel % g_rowh != 0) return;
    int c = rel / g_rowh;
    if (c >= NUM_CH) return;

    if (m.x >= g_x0 && m.x < g_x0 + LABEL_W) {
        if (m.bstate & BUTTON1_PRESSED) { cur_ch = c; toggle_mute(c); }
        return;
    }
    for (int s = 0; s < NUM_STEPS; s++) {
        if (m.x >= cell_x(s) && m.x < cell_x(s) + g_w) {
            cur_ch = c; cur_st = s;
            if (m.bstate & BUTTON1_PRESSED)      toggle_step(c, s);
            else if (m.bstate & BUTTON3_PRESSED) toggle_accent(c, s);
            else if (m.bstate & BUTTON2_PRESSED) atomic_store(&ch_len[c], s + 1);
            return;
        }
    }
}

static void handle_key(int k)
{
    switch (k) {
    case KEY_LEFT:  case 'h': if (cur_st > 0) cur_st--; break;
    case KEY_RIGHT: case 'l': if (cur_st < NUM_STEPS - 1) cur_st++; break;
    case KEY_UP:    case 'k': if (cur_ch > 0) cur_ch--; break;
    case KEY_DOWN:  case 'j': if (cur_ch < NUM_CH - 1) cur_ch++; break;
    case 'x': case '\n': case '\r': case KEY_ENTER: toggle_step(cur_ch, cur_st); break;
    case 'a': toggle_accent(cur_ch, cur_st); break;
    case 'e': atomic_store(&ch_len[cur_ch], cur_st + 1); break;
    case 'E': atomic_store(&ch_len[cur_ch], NUM_STEPS); break;
    case 'm': toggle_mute(cur_ch); break;
    case '1': case '2': case '3': case '4': toggle_mute(k - '1'); break;
    case 'c': clear_row(cur_ch); break;
    case 'C': for (int c = 0; c < NUM_CH; c++) clear_row(c); break;
    case ' ': toggle_play(); break;
    case '+': case '=': clamp_bpm(atomic_load(&bpm) + 1); break;
    case '-': case '_': clamp_bpm(atomic_load(&bpm) - 1); break;
    case '>': case '.': clamp_bpm(atomic_load(&bpm) + 10); break;
    case '<': case ',': clamp_bpm(atomic_load(&bpm) - 10); break;
    case 's': clamp_swing(atomic_load(&swing_pct) + 1); break;
    case 'S': clamp_swing(atomic_load(&swing_pct) - 1); break;
    case 'v': clamp_accent(atomic_load(&accent_amt) + 1); break;
    case 'V': clamp_accent(atomic_load(&accent_amt) - 1); break;
    case 'q': running = 0; break;
    case KEY_MOUSE: handle_mouse(); break;
    default: break;
    }
}

int main(int argc, char **argv)
{
    int kick[] = {0, 4, 8, 12}, snare[] = {4, 12}, hat[] = {2, 6, 10, 14}, aux[] = {7, 15};
    for (int c = 0; c < NUM_CH; c++) { atomic_store(&ch_len[c], NUM_STEPS); atomic_store(&cur_step[c], -1); }
    for (int i = 0; i < 4; i++) pattern[0][kick[i]] = 1;
    for (int i = 0; i < 2; i++) pattern[1][snare[i]] = 2;
    for (int i = 0; i < 4; i++) pattern[2][hat[i]] = 1;
    for (int i = 0; i < 2; i++) pattern[3][aux[i]] = 1;

    if (argc > 2) {
        int ch = atoi(argv[2]);
        if (ch >= 1 && ch <= 16) midi_channel = ch - 1;
    }

    /* JACK */
    jack_set_error_function(jack_quiet);
    jack_set_info_function(jack_quiet);
    jack_status_t st;
    client = jack_client_open("drumseq", JackNullOption, &st);
    if (!client) { fprintf(stderr, "Could not connect to JACK (is jackd running?)\n"); return 1; }
    out_port   = jack_port_register(client, "midi_out",   JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
    clock_port = jack_port_register(client, "clock_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
    jack_set_process_callback(client, process, NULL);
    if (jack_activate(client)) { fprintf(stderr, "Cannot activate JACK client\n"); return 1; }

    if (argc > 1 && jack_connect(client, jack_port_name(out_port), argv[1]))
        fprintf(stderr, "Could not connect to %s\n", argv[1]);
    if (argc > 3 && jack_connect(client, jack_port_name(clock_port), argv[3]))
        fprintf(stderr, "Could not connect clock to %s\n", argv[3]);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* ncurses */
    initscr();
    cbreak();
    noecho();
    nonl();
    curs_set(0);
    keypad(stdscr, TRUE);
    set_escdelay(25);
    timeout(20);                       /* ~50 wakeups/s max, idle otherwise */
    init_colors();
    mousemask(BUTTON1_PRESSED | BUTTON2_PRESSED | BUTTON3_PRESSED
#ifdef BUTTON4_PRESSED
              | BUTTON4_PRESSED
#endif
#ifdef BUTTON5_PRESSED
              | BUTTON5_PRESSED
#endif
              , NULL);
    mouseinterval(0);

    int dirty = 1, last_play = -1, last_step[NUM_CH];
    for (int c = 0; c < NUM_CH; c++) last_step[c] = -2;

    while (running) {
        int k = getch();
        if (k != ERR) { if (k != KEY_RESIZE) handle_key(k); dirty = 1; }

        /* redraw only when something changed */
        int p = atomic_load(&playing);
        if (p != last_play) { last_play = p; dirty = 1; }
        for (int c = 0; c < NUM_CH; c++) {
            int s = atomic_load(&cur_step[c]);
            if (s != last_step[c]) { last_step[c] = s; dirty = 1; }
        }
        if (dirty) { draw(); dirty = 0; }
    }

    endwin();
    atomic_store(&playing, 0);
    usleep(60000);                     /* let the callback flush note-offs / send Stop */
    jack_client_close(client);
    return 0;
}
