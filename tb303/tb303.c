/*
 * tb303.c - TB-303 style monophonic bass synth + 16 step piano roll
 *
 * Build (Debian 11):
 *   sudo apt install build-essential libjack-jackd2-dev libsdl2-dev jackd2
 *   gcc -O2 -Wall tb303.c -o tb303 $(sdl2-config --cflags --libs) -ljack -lm
 *
 * Run:  start jackd (or qjackctl) first, then ./tb303
 *
 * Controls:
 *   Mouse on piano roll : click/drag to place notes (one per step, monophonic)
 *                         click an existing note to erase it (drag to erase more)
 *   ACC / SLD lanes     : click to toggle accent / slide per step
 *   Sliders             : drag with mouse
 *   SPACE play/stop, R randomize pattern, C clear pattern, ESC quit
 *   M toggles tempo source: INT (tempo slider) / EXT (MIDI clock)
 *
 * External sync: connect a MIDI clock source to the "tb303:midi_in" JACK port.
 * In EXT mode the sequencer follows 0xF8 clock (24 ppqn, 6 ticks per step),
 * 0xFA start, 0xFB continue and 0xFC stop. The tempo slider is ignored.
 * ALSA hardware ports: run jackd with "-X seq" or use a2jmidid -e.
 */

#include <SDL2/SDL.h>
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

/* ---------- tiny 3x5 bitmap font ---------- */
#define G(a,b,c,d,e) (((a)<<12)|((b)<<9)|((c)<<6)|((d)<<3)|(e))
static const int font_alpha[26] = {
    G(2,5,7,5,5), G(6,5,6,5,6), G(3,4,4,4,3), G(6,5,5,5,6), G(7,4,6,4,7),
    G(7,4,6,4,4), G(3,4,5,5,3), G(5,5,7,5,5), G(7,2,2,2,7), G(1,1,1,5,2),
    G(5,5,6,5,5), G(4,4,4,4,7), G(5,7,7,5,5), G(6,5,5,5,5), G(2,5,5,5,2),
    G(6,5,6,4,4), G(2,5,5,7,3), G(6,5,6,5,5), G(3,4,2,1,6), G(7,2,2,2,2),
    G(5,5,5,5,7), G(5,5,5,5,2), G(5,5,7,7,5), G(5,5,2,5,5), G(5,5,2,2,2),
    G(7,1,2,4,7) };
static const int font_digit[10] = {
    G(7,5,5,5,7), G(2,6,2,2,7), G(6,1,2,4,7), G(6,1,2,1,6), G(5,5,7,1,1),
    G(7,4,6,1,6), G(3,4,7,5,7), G(7,1,1,2,2), G(7,5,7,5,7), G(7,5,7,1,6) };

static int glyph(char c)
{
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return font_alpha[c - 'A'];
    if (c >= '0' && c <= '9') return font_digit[c - '0'];
    if (c == '#') return G(5,7,5,7,5);
    if (c == '/') return G(1,1,2,4,4);
    if (c == '-') return G(0,0,7,0,0);
    return 0;
}

static void rect(SDL_Renderer *r, int x, int y, int w, int h,
                 int R, int Gc, int B)
{
    SDL_SetRenderDrawColor(r, R, Gc, B, 255);
    SDL_Rect rc = { x, y, w, h };
    SDL_RenderFillRect(r, &rc);
}

static void text(SDL_Renderer *r, int x, int y, int sc, const char *s,
                 int R, int Gc, int B)
{
    SDL_SetRenderDrawColor(r, R, Gc, B, 255);
    for (; *s; s++, x += 4 * sc) {
        int g = glyph(*s);
        for (int row = 0; row < 5; row++)
            for (int col = 0; col < 3; col++)
                if (g & (1 << (14 - row * 3 - col))) {
                    SDL_Rect rc = { x + col * sc, y + row * sc, sc, sc };
                    SDL_RenderFillRect(r, &rc);
                }
    }
}

/* ---------- layout ---------- */
#define WIN_W 880
#define WIN_H 580
#define RX 50
#define RY 130
#define CW 50
#define RH 14
#define ACC_Y (RY + ROWS * RH + 8)
#define SLD_Y (ACC_Y + 22)
#define LANE_H 18

static void slider_rect(int i, SDL_Rect *rc)
{
    rc->x = 20 + (i % 4) * 215;
    rc->y = 40 + (i / 4) * 45;
    rc->w = 190;
    rc->h = 14;
}

static int is_black(int pc) { return pc == 1 || pc == 3 || pc == 6 || pc == 8 || pc == 10; }

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

static int roll_hit(int mx, int my, int *col, int *note)
{
    if (mx < RX || mx >= RX + STEPS * CW || my < RY || my >= RY + ROWS * RH) return 0;
    *col = (mx - RX) / CW;
    *note = ROWS - 1 - (my - RY) / RH;
    return 1;
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

    /* SDL */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) { fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1; }
    SDL_Window *win = SDL_CreateWindow("TB-303 clone", SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
                                           SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);

    int drag_slider = -1;
    int paint = 0;          /* 0 none, 1 set, 2 erase */

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_QUIT: running = 0; break;
            case SDL_KEYDOWN:
                if (e.key.keysym.sym == SDLK_ESCAPE) running = 0;
                else if (e.key.keysym.sym == SDLK_SPACE) playing = !playing;
                else if (e.key.keysym.sym == SDLK_r) randomize_pattern();
                else if (e.key.keysym.sym == SDLK_c) memset(steps, 0, sizeof steps);
                else if (e.key.keysym.sym == SDLK_m) ext_sync = !ext_sync;
                break;
            case SDL_MOUSEBUTTONDOWN: {
                int mx = e.button.x, my = e.button.y, col, note;
                for (int i = 0; i < P_COUNT; i++) {
                    SDL_Rect rc; slider_rect(i, &rc);
                    if (mx >= rc.x && mx < rc.x + rc.w && my >= rc.y - 4 && my < rc.y + rc.h + 4) {
                        drag_slider = i;
                        float v = (float)(mx - rc.x) / rc.w;
                        param[i] = v < 0 ? 0 : v > 1 ? 1 : v;
                    }
                }
                if (roll_hit(mx, my, &col, &note)) {
                    paint = (steps[col].gate && steps[col].note == note) ? 2 : 1;
                    if (paint == 1) { steps[col].note = note; steps[col].gate = 1; }
                    else steps[col].gate = 0;
                } else if (mx >= RX && mx < RX + STEPS * CW) {
                    col = (mx - RX) / CW;
                    if (my >= ACC_Y && my < ACC_Y + LANE_H) steps[col].accent ^= 1;
                    if (my >= SLD_Y && my < SLD_Y + LANE_H) steps[col].slide ^= 1;
                }
                break; }
            case SDL_MOUSEMOTION: {
                int mx = e.motion.x, my = e.motion.y, col, note;
                if (drag_slider >= 0) {
                    SDL_Rect rc; slider_rect(drag_slider, &rc);
                    float v = (float)(mx - rc.x) / rc.w;
                    param[drag_slider] = v < 0 ? 0 : v > 1 ? 1 : v;
                }
                if (paint && roll_hit(mx, my, &col, &note)) {
                    if (paint == 1) { steps[col].note = note; steps[col].gate = 1; }
                    else steps[col].gate = 0;
                }
                break; }
            case SDL_MOUSEBUTTONUP:
                drag_slider = -1; paint = 0; break;
            }
        }

        /* ---- draw ---- */
        rect(ren, 0, 0, WIN_W, WIN_H, 28, 28, 32);
        text(ren, 20, 10, 3, "TB-303 CLONE", 230, 120, 40);
        text(ren, 300, 14, 2, playing ? "PLAYING" : "STOPPED",
             playing ? 90 : 160, playing ? 220 : 160, playing ? 90 : 160);

        {
            char sb[32];
            if (ext_sync) snprintf(sb, sizeof sb, "SYNC EXT %d", (int)(ext_bpm + 0.5f));
            else snprintf(sb, sizeof sb, "SYNC INT");
            text(ren, 450, 14, 2, sb, ext_sync ? 90 : 160, ext_sync ? 200 : 160, ext_sync ? 230 : 160);
        }

        for (int i = 0; i < P_COUNT; i++) {
            SDL_Rect rc; slider_rect(i, &rc);
            char buf[32];
            text(ren, rc.x, rc.y - 13, 2, param_name[i], 200, 200, 200);
            if (i == P_TEMPO) snprintf(buf, sizeof buf, "%d",
                                       ext_sync ? (int)(ext_bpm + 0.5f) : (int)(60 + param[i] * 140));
            else if (i == P_WAVE) snprintf(buf, sizeof buf, "%s", param[i] > 0.5f ? "SQR" : "SAW");
            else snprintf(buf, sizeof buf, "%d", (int)(param[i] * 100));
            text(ren, rc.x + 120, rc.y - 13, 2, buf, 230, 120, 40);
            rect(ren, rc.x, rc.y, rc.w, rc.h, 55, 55, 62);
            rect(ren, rc.x, rc.y, (int)(rc.w * param[i]), rc.h, 210, 100, 30);
        }

        /* piano roll */
        int cs = cur_step;
        for (int row = 0; row < ROWS; row++) {
            int n = ROWS - 1 - row;
            int pc = (BASE_MIDI + n) % 12;
            int y = RY + row * RH;
            int bk = is_black(pc);
            for (int c = 0; c < STEPS; c++) {
                int shade = bk ? 38 : 50;
                if ((c / 4) % 2) shade += 8;
                if (c == cs) shade += 18;
                rect(ren, RX + c * CW, y, CW - 1, RH - 1, shade, shade, shade + 6);
            }
            if (pc == 0) {
                char lb[8];
                snprintf(lb, sizeof lb, "C%d", (BASE_MIDI + n) / 12 - 1);
                text(ren, 14, y + 2, 2, lb, 230, 120, 40);
            }
        }
        for (int c = 0; c < STEPS; c++) {
            if (steps[c].gate) {
                int y = RY + (ROWS - 1 - steps[c].note) * RH;
                int w = CW - 1;
                if (steps[c].slide) w = CW; /* visually tie into next step */
                rect(ren, RX + c * CW, y, w, RH - 1,
                     steps[c].accent ? 255 : 220, steps[c].accent ? 60 : 130, 30);
            }
            /* lanes */
            rect(ren, RX + c * CW, ACC_Y, CW - 1, LANE_H,
                 steps[c].accent ? 230 : 50, steps[c].accent ? 50 : 50, steps[c].accent ? 50 : 56);
            rect(ren, RX + c * CW, SLD_Y, CW - 1, LANE_H,
                 steps[c].slide ? 60 : 50, steps[c].slide ? 160 : 50, steps[c].slide ? 230 : 56);
            char num[8]; snprintf(num, sizeof num, "%d", c + 1);
            text(ren, RX + c * CW + 4, SLD_Y + LANE_H + 6, 2, num,
                 c == cs ? 255 : 120, c == cs ? 255 : 120, c == cs ? 255 : 130);
        }
        text(ren, 6, ACC_Y + 3, 2, "ACC", 230, 80, 80);
        text(ren, 6, SLD_Y + 3, 2, "SLD", 80, 160, 230);
        text(ren, 20, WIN_H - 24, 2,
             "SPACE PLAY/STOP  R RANDOM  C CLEAR  M SYNC INT/EXT  ESC QUIT", 140, 140, 150);

        SDL_RenderPresent(ren);
    }

    jack_client_close(client);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
