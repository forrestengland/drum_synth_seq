/*
 * drumseq.c - 4 channel / 16 step drum sequencer
 *             SDL2 UI + JACK MIDI output
 *
 * Build:
 *   gcc -O2 -Wall -o drumseq drumseq.c $(pkg-config --cflags --libs sdl2 jack) -lm
 *
 * Run:
 *   ./drumseq [target_jack_midi_port] [midi_channel 1-16] [clock_target_port]
 *   e.g. ./drumseq drumsynth:midi_in 1 otherapp:midi_in
 *
 * Ports: drumseq:midi_out (notes), drumseq:clock_out (MIDI clock + start/stop)
 *
 * Controls:
 *   Left click / drag    toggle steps (normal velocity)
 *   Right click / drag   toggle accent steps (full velocity)
 *   Middle click         set last step of channel (polyrhythms)
 *   Click channel label  mute / unmute channel
 *   Space                play / stop
 *   Up / Down            BPM +/- 1  (hold Shift for 10)
 *   Left / Right         swing -/+ 1%
 *   PgUp / PgDn          accent amount +/- 1 (Shift for 10)
 *   Mouse wheel          BPM
 *   1-4                  mute channels
 *   C                    clear pattern
 *   Esc                  quit
 */

#include <SDL2/SDL.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
/* Tiny 3x5 bitmap font (no SDL_ttf dependency)                       */
/* ------------------------------------------------------------------ */
static const struct { char c; const char *p; } font[] = {
    {'0',"111101101101111"},{'1',"010110010010111"},{'2',"111001111100111"},
    {'3',"111001111001111"},{'4',"101101111001001"},{'5',"111100111001111"},
    {'6',"111100111101111"},{'7',"111001001001001"},{'8',"111101111101111"},
    {'9',"111101111001111"},{'A',"010101111101101"},{'B',"110101110101110"},
    {'C',"011100100100011"},{'E',"111100110100111"},{'G',"011100101101011"},
    {'H',"101101111101101"},{'I',"111010010010111"},{'L',"100100100100111"},
    {'M',"101111111101101"},{'N',"110101101101101"},{'O',"010101101101010"},
    {'P',"110101110100100"},{'R',"110101110101101"},{'S',"011100010001110"},
    {'T',"111010010010010"},{'U',"101101101101111"},{'W',"101101111111101"},
    {'Y',"101101010010010"},{'+',"000010111010000"},{'-',"000000111000000"},
    {'%',"101001010100101"},
};

static void draw_text(SDL_Renderer *r, int x, int y, int s, const char *str)
{
    for (; *str; str++, x += 4 * s) {
        for (size_t f = 0; f < sizeof font / sizeof font[0]; f++) {
            if (font[f].c != *str) continue;
            for (int i = 0; i < 15; i++) {
                if (font[f].p[i] == '1') {
                    SDL_Rect q = { x + (i % 3) * s, y + (i / 3) * s, s, s };
                    SDL_RenderFillRect(r, &q);
                }
            }
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* UI                                                                 */
/* ------------------------------------------------------------------ */
#define WIN_W 990
#define WIN_H 340
#define GRID_X 120
#define GRID_Y 110
#define CELL   48
#define PITCH  52
#define GROUP_GAP 10
#define ROW_PITCH 58

static const SDL_Color ch_col[NUM_CH] = {
    {235, 90, 70, 255}, {245, 190, 50, 255}, {80, 200, 120, 255}, {80, 150, 240, 255}
};

static const SDL_Rect R_PLAY   = { 20,  20, 80, 40 };
static const SDL_Rect R_BPM_M  = { 215, 20, 36, 40 };
static const SDL_Rect R_BPM_P  = { 259, 20, 36, 40 };
static const SDL_Rect R_SW_M   = { 450, 20, 36, 40 };
static const SDL_Rect R_SW_P   = { 494, 20, 36, 40 };
static const SDL_Rect R_AC_M   = { 670, 20, 36, 40 };
static const SDL_Rect R_AC_P   = { 714, 20, 36, 40 };
static const SDL_Rect R_CLEAR  = { 780, 20, 70, 40 };

static int in_rect(const SDL_Rect *r, int x, int y)
{
    return x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h;
}

static int cell_x(int s) { return GRID_X + s * PITCH + (s / 4) * GROUP_GAP; }
static int cell_y(int c) { return GRID_Y + c * ROW_PITCH; }
static SDL_Rect label_rect(int c) { SDL_Rect r = { 20, cell_y(c), 88, CELL }; return r; }

static int hit_cell(int x, int y, int *ch, int *st)
{
    for (int c = 0; c < NUM_CH; c++) {
        if (y < cell_y(c) || y >= cell_y(c) + CELL) continue;
        for (int s = 0; s < NUM_STEPS; s++)
            if (x >= cell_x(s) && x < cell_x(s) + CELL) { *ch = c; *st = s; return 1; }
    }
    return 0;
}

static void set_col(SDL_Renderer *r, SDL_Color c, float k)
{
    SDL_SetRenderDrawColor(r, (Uint8)(c.r * k), (Uint8)(c.g * k), (Uint8)(c.b * k), 255);
}

static void button(SDL_Renderer *r, SDL_Rect rc, const char *label, int active)
{
    SDL_SetRenderDrawColor(r, active ? 70 : 50, active ? 120 : 52, active ? 70 : 58, 255);
    SDL_RenderFillRect(r, &rc);
    SDL_SetRenderDrawColor(r, 120, 120, 130, 255);
    SDL_RenderDrawRect(r, &rc);
    int w = (int)strlen(label) * 12 - 3;
    SDL_SetRenderDrawColor(r, 230, 230, 235, 255);
    draw_text(r, rc.x + (rc.w - w) / 2, rc.y + (rc.h - 15) / 2, 3, label);
}

static void render(SDL_Renderer *r)
{
    char buf[32];
    SDL_SetRenderDrawColor(r, 24, 24, 28, 255);
    SDL_RenderClear(r);

    /* top bar */
    button(r, R_PLAY, atomic_load(&playing) ? "STOP" : "PLAY", atomic_load(&playing));
    SDL_SetRenderDrawColor(r, 230, 230, 235, 255);
    snprintf(buf, sizeof buf, "BPM %d", atomic_load(&bpm));
    draw_text(r, 120, 33, 3, buf);
    button(r, R_BPM_M, "-", 0);
    button(r, R_BPM_P, "+", 0);
    SDL_SetRenderDrawColor(r, 230, 230, 235, 255);
    snprintf(buf, sizeof buf, "SWING %d", atomic_load(&swing_pct));
    draw_text(r, 330, 33, 3, buf);
    button(r, R_SW_M, "-", 0);
    button(r, R_SW_P, "+", 0);
    SDL_SetRenderDrawColor(r, 230, 230, 235, 255);
    snprintf(buf, sizeof buf, "ACCENT %d", atomic_load(&accent_amt));
    draw_text(r, 550, 33, 3, buf);
    button(r, R_AC_M, "-", 0);
    button(r, R_AC_P, "+", 0);
    button(r, R_CLEAR, "CLR", 0);

    for (int c = 0; c < NUM_CH; c++) {
        int mute = atomic_load(&muted[c]);
        int step = atomic_load(&cur_step[c]);
        int len  = atomic_load(&ch_len[c]);
        float k = mute ? 0.30f : 1.0f;

        /* label */
        SDL_Rect lr = label_rect(c);
        set_col(r, ch_col[c], mute ? 0.25f : 0.9f);
        SDL_RenderFillRect(r, &lr);
        SDL_SetRenderDrawColor(r, 15, 15, 18, 255);
        snprintf(buf, sizeof buf, "CH%d", c + 1);
        draw_text(r, lr.x + 10, lr.y + 8, 3, buf);
        snprintf(buf, sizeof buf, "N%d", CH_NOTE[c]);
        draw_text(r, lr.x + 10, lr.y + 28, 2, buf);

        for (int s = 0; s < NUM_STEPS; s++) {
            SDL_Rect cr = { cell_x(s), cell_y(c), CELL, CELL };
            int v = pattern[c][s];
            int out = s >= len;                 /* beyond loop end */

            if (v == 0) {
                int shade = out ? 28 : (((s / 4) & 1) ? 48 : 38);
                SDL_SetRenderDrawColor(r, shade, shade, shade + 6, 255);
            } else {
                set_col(r, ch_col[c], (v == 2 ? 1.0f : 0.72f) * k * (out ? 0.2f : 1.0f));
            }
            SDL_RenderFillRect(r, &cr);

            if (v == 2) {                       /* accent marker */
                SDL_Rect m = { cr.x + CELL / 2 - 4, cr.y + CELL / 2 - 4, 8, 8 };
                SDL_SetRenderDrawColor(r, 255, 255, 255, (Uint8)(mute ? 90 : 255));
                SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
                SDL_RenderFillRect(r, &m);
            }
            if (s == len - 1) {                 /* last-step marker */
                SDL_Rect lm = { cr.x + CELL - 5, cr.y, 5, CELL };
                set_col(r, ch_col[c], mute ? 0.5f : 1.0f);
                SDL_RenderFillRect(r, &lm);
                SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
                SDL_Rect lt = { cr.x + CELL - 5, cr.y, 5, 4 };
                SDL_Rect lb = { cr.x + CELL - 5, cr.y + CELL - 4, 5, 4 };
                SDL_RenderFillRect(r, &lt);
                SDL_RenderFillRect(r, &lb);
            }
            if (s == step) {                    /* playhead */
                SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
                SDL_RenderDrawRect(r, &cr);
                SDL_Rect in = { cr.x + 1, cr.y + 1, CELL - 2, CELL - 2 };
                SDL_RenderDrawRect(r, &in);
            }
        }
    }

    SDL_RenderPresent(r);
}

/* ------------------------------------------------------------------ */
static void clamp_bpm(int v)   { atomic_store(&bpm, v < 40 ? 40 : v > 240 ? 240 : v); }
static void clamp_swing(int v) { atomic_store(&swing_pct, v < 0 ? 0 : v > 50 ? 50 : v); }
static void clamp_accent(int v) { atomic_store(&accent_amt, v < 0 ? 0 : v > 100 ? 100 : v); }
static void clear_pattern(void) { memset((void *)pattern, 0, sizeof pattern); }
static void toggle_play(void)   { atomic_store(&playing, !atomic_load(&playing)); }

int main(int argc, char **argv)
{
    for (int c = 0; c < NUM_CH; c++) { atomic_store(&ch_len[c], NUM_STEPS); atomic_store(&cur_step[c], -1); }

    /* default beat */
    int kick[] = {0, 4, 8, 12}, snare[] = {4, 12}, hat[] = {2, 6, 10, 14}, aux[] = {7, 15};
    for (int i = 0; i < 4; i++) pattern[0][kick[i]] = 1;
    for (int i = 0; i < 2; i++) pattern[1][snare[i]] = 2;
    for (int i = 0; i < 4; i++) pattern[2][hat[i]] = 1;
    for (int i = 0; i < 2; i++) pattern[3][aux[i]] = 1;

    if (argc > 2) {
        int ch = atoi(argv[2]);
        if (ch >= 1 && ch <= 16) midi_channel = ch - 1;
    }

    /* JACK */
    jack_status_t st;
    client = jack_client_open("drumseq", JackNullOption, &st);
    if (!client) { fprintf(stderr, "Could not connect to JACK\n"); return 1; }
    out_port = jack_port_register(client, "midi_out", JACK_DEFAULT_MIDI_TYPE,
                                  JackPortIsOutput, 0);
    clock_port = jack_port_register(client, "clock_out", JACK_DEFAULT_MIDI_TYPE,
                                    JackPortIsOutput, 0);
    jack_set_process_callback(client, process, NULL);
    if (jack_activate(client)) { fprintf(stderr, "Cannot activate JACK client\n"); return 1; }

    if (argc > 1) {
        if (jack_connect(client, jack_port_name(out_port), argv[1]))
            fprintf(stderr, "Could not connect to %s\n", argv[1]);
        else
            printf("Connected to %s\n", argv[1]);
    }

    if (argc > 3) {
        if (jack_connect(client, jack_port_name(clock_port), argv[3]))
            fprintf(stderr, "Could not connect clock to %s\n", argv[3]);
        else
            printf("Clock connected to %s\n", argv[3]);
    }

    /* SDL */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) { fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1; }
    SDL_Window *win = SDL_CreateWindow("drumseq", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                       WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    int running = 1, painting = 0, paint_val = 0;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_QUIT: running = 0; break;

            case SDL_KEYDOWN: {
                int mod = (e.key.keysym.mod & KMOD_SHIFT) ? 10 : 1;
                switch (e.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_SPACE:  toggle_play(); break;
                case SDLK_UP:     clamp_bpm(atomic_load(&bpm) + mod); break;
                case SDLK_DOWN:   clamp_bpm(atomic_load(&bpm) - mod); break;
                case SDLK_RIGHT:  clamp_swing(atomic_load(&swing_pct) + 1); break;
                case SDLK_LEFT:   clamp_swing(atomic_load(&swing_pct) - 1); break;
                case SDLK_PAGEUP:   clamp_accent(atomic_load(&accent_amt) + mod); break;
                case SDLK_PAGEDOWN: clamp_accent(atomic_load(&accent_amt) - mod); break;
                case SDLK_c:      clear_pattern(); break;
                case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4: {
                    int c = e.key.keysym.sym - SDLK_1;
                    atomic_store(&muted[c], !atomic_load(&muted[c]));
                    break;
                }
                }
                break;
            }

            case SDL_MOUSEWHEEL:
                clamp_bpm(atomic_load(&bpm) + e.wheel.y);
                break;

            case SDL_MOUSEBUTTONDOWN: {
                int x = e.button.x, y = e.button.y, ch, s;
                if (hit_cell(x, y, &ch, &s)) {
                    int cur = pattern[ch][s];
                    if (e.button.button == SDL_BUTTON_LEFT)       paint_val = cur == 0 ? 1 : 0;
                    else if (e.button.button == SDL_BUTTON_RIGHT) paint_val = cur == 2 ? 0 : 2;
                    else if (e.button.button == SDL_BUTTON_MIDDLE) {
                        atomic_store(&ch_len[ch], s + 1);   /* set last step */
                        break;
                    } else break;
                    pattern[ch][s] = paint_val;
                    painting = e.button.button;
                } else if (e.button.button == SDL_BUTTON_LEFT) {
                    if (in_rect(&R_PLAY, x, y))       toggle_play();
                    else if (in_rect(&R_BPM_M, x, y)) clamp_bpm(atomic_load(&bpm) - 1);
                    else if (in_rect(&R_BPM_P, x, y)) clamp_bpm(atomic_load(&bpm) + 1);
                    else if (in_rect(&R_SW_M, x, y))  clamp_swing(atomic_load(&swing_pct) - 1);
                    else if (in_rect(&R_SW_P, x, y))  clamp_swing(atomic_load(&swing_pct) + 1);
                    else if (in_rect(&R_AC_M, x, y))  clamp_accent(atomic_load(&accent_amt) - 1);
                    else if (in_rect(&R_AC_P, x, y))  clamp_accent(atomic_load(&accent_amt) + 1);
                    else if (in_rect(&R_CLEAR, x, y)) clear_pattern();
                    else for (int c = 0; c < NUM_CH; c++) {
                        SDL_Rect lr = label_rect(c);
                        if (in_rect(&lr, x, y)) atomic_store(&muted[c], !atomic_load(&muted[c]));
                    }
                }
                break;
            }

            case SDL_MOUSEBUTTONUP:
                painting = 0;
                break;

            case SDL_MOUSEMOTION:
                if (painting) {
                    int ch, s;
                    if (hit_cell(e.motion.x, e.motion.y, &ch, &s))
                        pattern[ch][s] = paint_val;
                }
                break;
            }
        }
        render(ren);
    }

    atomic_store(&playing, 0);
    SDL_Delay(50);                 /* let the callback flush note-offs */
    jack_client_close(client);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
