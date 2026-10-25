/*
 * drumvoice - tiny analog-style drum voice synth
 *   JACK MIDI in -> 4 voices (BD, SD, closed hat, open hat) -> JACK stereo out
 *   SDL2 knob UI for tune / decay / bend / tone / level per voice.
 *
 * Default MIDI notes (GM): BD 36, SD 38, CH 42, OH 46
 * Override: ./drumvoice -b 36 -s 38 -c 42 -o 46 -n clientname
 *
 * Incoming note-ons and CCs are printed to stdout, so you can find the numbers
 * your controller sends. Then map them:
 *   -C t,d,b,x,l   five CC numbers -> TUNE, DECAY, BEND, tone(CLICK/NOISE/RESO), LEVEL
 *   -S a,b,c,d     four note numbers -> select BD, SD, CH, OH as the voice the
 *                  five CCs control (use -1 to leave a slot unmapped)
 * e.g. ./drumvoice -C 74,71,76,77,7 -S 40,41,43,44
 *
 * UI: drag a knob vertically (hold SHIFT for fine), mouse wheel over a knob,
 *     right-click a knob to reset it. Click a pad (or press 1-4) to audition.
 */

#define _GNU_SOURCE
#include <SDL2/SDL.h>
#include <jack/jack.h>
#include <jack/midiport.h>
#include <ctype.h>
#include <math.h>
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
static int cc_map[NP]    = { -1, -1, -1, -1, -1 };   /* CC number per param, -1 = unmapped */
static int sel_note[NV]  = { -1, -1, -1, -1 };       /* note that selects each voice       */
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
/* Tiny 5x7 bitmap font (column-major, bit0 = top row)                 */
/* ------------------------------------------------------------------ */
static const struct { char c; uint8_t d[5]; } font[] = {
    {' ',{0x00,0x00,0x00,0x00,0x00}}, {'-',{0x08,0x08,0x08,0x08,0x08}},
    {'.',{0x00,0x60,0x60,0x00,0x00}}, {'+',{0x08,0x08,0x3E,0x08,0x08}},
    {'%',{0x23,0x13,0x08,0x64,0x62}},
    {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
    {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
    {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
    {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
    {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
    {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
    {'E',{0x7F,0x49,0x49,0x49,0x41}}, {'F',{0x7F,0x09,0x09,0x09,0x01}},
    {'G',{0x3E,0x41,0x49,0x49,0x7A}}, {'H',{0x7F,0x08,0x08,0x08,0x7F}},
    {'I',{0x00,0x41,0x7F,0x41,0x00}}, {'J',{0x20,0x40,0x41,0x3F,0x01}},
    {'K',{0x7F,0x08,0x14,0x22,0x41}}, {'L',{0x7F,0x40,0x40,0x40,0x40}},
    {'M',{0x7F,0x02,0x0C,0x02,0x7F}}, {'N',{0x7F,0x04,0x08,0x10,0x7F}},
    {'O',{0x3E,0x41,0x41,0x41,0x3E}}, {'P',{0x7F,0x09,0x09,0x09,0x06}},
    {'Q',{0x3E,0x41,0x51,0x21,0x5E}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
    {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
    {'U',{0x3F,0x40,0x40,0x40,0x3F}}, {'V',{0x1F,0x20,0x40,0x20,0x1F}},
    {'W',{0x3F,0x40,0x38,0x40,0x3F}}, {'X',{0x63,0x14,0x08,0x14,0x63}},
    {'Y',{0x07,0x08,0x70,0x08,0x07}}, {'Z',{0x61,0x51,0x49,0x45,0x43}},
};

static int text_w(const char *s, int sc) { return (int)strlen(s) * 6 * sc - sc; }

static void draw_text(SDL_Renderer *r, int x, int y, int sc, const char *s)
{
    for (; *s; s++, x += 6 * sc) {
        char c = (char)toupper((unsigned char)*s);
        for (size_t g = 0; g < sizeof font / sizeof font[0]; g++) {
            if (font[g].c != c) continue;
            for (int col = 0; col < 5; col++)
                for (int row = 0; row < 7; row++)
                    if (font[g].d[col] >> row & 1) {
                        SDL_Rect q = { x + col * sc, y + row * sc, sc, sc };
                        SDL_RenderFillRect(r, &q);
                    }
            break;
        }
    }
}

static void draw_text_c(SDL_Renderer *r, int cx, int y, int sc, const char *s)
{
    draw_text(r, cx - text_w(s, sc) / 2, y, sc, s);
}

/* ------------------------------------------------------------------ */
/* UI                                                                  */
/* ------------------------------------------------------------------ */
#define WIN_W   640
#define ROW_H   115
#define TOP     10
#define KNOB_R  28
#define PAD_X   15
#define PAD_W   110
#define PAD_H   80
#define WIN_H   (TOP * 2 + NV * ROW_H + 12)

static void knob_pos(int v, int k, int *cx, int *cy)
{
    *cx = 180 + k * 100;
    *cy = TOP + v * ROW_H + 52;
}

static void pad_rect(int v, SDL_Rect *q)
{
    q->x = PAD_X; q->y = TOP + v * ROW_H + 14; q->w = PAD_W; q->h = PAD_H;
}

static void fill_circle(SDL_Renderer *r, int cx, int cy, int rad)
{
    for (int dy = -rad; dy <= rad; dy++) {
        int dx = (int)sqrtf((float)(rad * rad - dy * dy));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

static void knob_angle(float n, float *c, float *s)
{
    float a = (225.f - 270.f * n) * (PI / 180.f);
    *c = cosf(a); *s = sinf(a);
}

static void param_text(int v, int k, char *buf, size_t sz)
{
    const VoiceDef *d = &defs[v];
    float n = clamp01(P[v][k]);
    switch (k) {
    case P_TUNE:  snprintf(buf, sz, "%.0f HZ", expmap(n, d->tune_lo, d->tune_hi)); break;
    case P_DECAY: snprintf(buf, sz, "%.0f MS", 1000.f * expmap(n, d->dec_lo, d->dec_hi)); break;
    case P_BEND:  snprintf(buf, sz, "X%.1f", 1.f + n * d->bend_max); break;
    default:      snprintf(buf, sz, "%.0f%%", n * 100.f); break;
    }
}

static void draw_ui(SDL_Renderer *r, const float *flash, const char *status)
{
    SDL_SetRenderDrawColor(r, 24, 24, 28, 255);
    SDL_RenderClear(r);

    for (int v = 0; v < NV; v++) {
        int rowy = TOP + v * ROW_H;

        /* marker for the voice the MIDI CC knobs currently control */
        if (v == atomic_load(&sel_voice)) {
            SDL_SetRenderDrawColor(r, 255, 160, 40, 255);
            SDL_Rect mk = { 3, rowy + 4, 5, ROW_H - 12 };
            SDL_RenderFillRect(r, &mk);
        }

        /* row separator */
        SDL_SetRenderDrawColor(r, 44, 44, 52, 255);
        SDL_RenderDrawLine(r, 10, rowy + ROW_H - 4, WIN_W - 10, rowy + ROW_H - 4);

        /* pad */
        SDL_Rect pr; pad_rect(v, &pr);
        int base = 50 + (int)(flash[v] * 170.f);
        SDL_SetRenderDrawColor(r, base, base * 2 / 3 + 20, 30, 255);
        SDL_RenderFillRect(r, &pr);
        SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
        draw_text_c(r, pr.x + pr.w / 2, pr.y + 12, 4, defs[v].pad);
        char nb[24]; snprintf(nb, sizeof nb, "NOTE %d", midi_note[v]);
        SDL_SetRenderDrawColor(r, 220, 220, 220, 255);
        draw_text_c(r, pr.x + pr.w / 2, pr.y + 52, 2, nb);

        /* knobs */
        for (int k = 0; k < NP; k++) {
            int cx, cy; knob_pos(v, k, &cx, &cy);
            float n = clamp01(P[v][k]);

            const char *name = (k == P_TONE) ? defs[v].tone_name : pname[k];
            SDL_SetRenderDrawColor(r, 170, 170, 180, 255);
            draw_text_c(r, cx, rowy + 6, 2, name);

            /* scale dots + value arc */
            for (int t = 0; t <= 40; t++) {
                float tn = t / 40.f, c, s;
                knob_angle(tn, &c, &s);
                int px = cx + (int)(c * (KNOB_R + 7)), py = cy - (int)(s * (KNOB_R + 7));
                if (tn <= n + 0.001f) SDL_SetRenderDrawColor(r, 255, 160, 40, 255);
                else                  SDL_SetRenderDrawColor(r, 70, 70, 80, 255);
                SDL_Rect dot = { px - 1, py - 1, 3, 3 };
                SDL_RenderFillRect(r, &dot);
            }

            /* body */
            SDL_SetRenderDrawColor(r, 20, 20, 24, 255);
            fill_circle(r, cx, cy, KNOB_R + 1);
            SDL_SetRenderDrawColor(r, 78, 80, 92, 255);
            fill_circle(r, cx, cy, KNOB_R - 2);
            SDL_SetRenderDrawColor(r, 96, 98, 112, 255);
            fill_circle(r, cx, cy, KNOB_R - 8);

            /* pointer */
            float c, s; knob_angle(n, &c, &s);
            SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
            for (int o = -1; o <= 1; o++)
                SDL_RenderDrawLine(r, cx + o, cy,
                                   cx + o + (int)(c * (KNOB_R - 3)),
                                   cy - (int)(s * (KNOB_R - 3)));

            /* value text */
            char vb[32]; param_text(v, k, vb, sizeof vb);
            SDL_SetRenderDrawColor(r, 255, 200, 120, 255);
            draw_text_c(r, cx, rowy + 90, 2, vb);
        }
    }
    SDL_SetRenderDrawColor(r, 150, 150, 160, 255);
    draw_text(r, 15, WIN_H - 20, 2, status);
    SDL_RenderPresent(r);
}

static int hit_knob(int mx, int my, int *vv, int *kk)
{
    for (int v = 0; v < NV; v++)
        for (int k = 0; k < NP; k++) {
            int cx, cy; knob_pos(v, k, &cx, &cy);
            int dx = mx - cx, dy = my - cy;
            if (dx * dx + dy * dy <= (KNOB_R + 5) * (KNOB_R + 5)) { *vv = v; *kk = k; return 1; }
        }
    return 0;
}

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

static const char *ch_names[NV] = { "BD", "SD", "CH", "OH" };

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

    /* SDL */
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        jack_client_close(client);
        return 1;
    }
    SDL_Window *win = SDL_CreateWindow("drumvoice", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                       WIN_W, WIN_H, 0);
    SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC) : NULL;
    if (win && !ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!win || !ren) {
        fprintf(stderr, "SDL window/renderer: %s\n", SDL_GetError());
        jack_client_close(client);
        SDL_Quit();
        return 1;
    }

    float flash[NV] = {0};
    char status[64] = "WAITING FOR MIDI";
    int prev_sel = 0;
    int drag_v = -1, drag_k = -1;
    float drag_y0 = 0.f, drag_n0 = 0.f;

    while (!atomic_load(&quit_flag)) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_QUIT:
                atomic_store(&quit_flag, 1);
                break;
            case SDL_KEYDOWN:
                if (e.key.repeat) break;
                if (e.key.keysym.sym == SDLK_ESCAPE) atomic_store(&quit_flag, 1);
                if (e.key.keysym.sym >= SDLK_1 && e.key.keysym.sym <= SDLK_4)
                    ui_trigger(e.key.keysym.sym - SDLK_1);
                break;
            case SDL_MOUSEBUTTONDOWN: {
                int v, k;
                if (e.button.button == SDL_BUTTON_LEFT) {
                    for (int i = 0; i < NV; i++) {
                        SDL_Rect pr; pad_rect(i, &pr);
                        SDL_Point p = { e.button.x, e.button.y };
                        if (SDL_PointInRect(&p, &pr)) { ui_trigger(i); atomic_store(&sel_voice, i); }
                    }
                    if (hit_knob(e.button.x, e.button.y, &v, &k)) {
                        drag_v = v; drag_k = k;
                        drag_y0 = (float)e.button.y;
                        drag_n0 = P[v][k];
                    }
                } else if (e.button.button == SDL_BUTTON_RIGHT) {
                    if (hit_knob(e.button.x, e.button.y, &v, &k))
                        P[v][k] = defs[v].def[k];
                }
                break;
            }
            case SDL_MOUSEBUTTONUP:
                if (e.button.button == SDL_BUTTON_LEFT) drag_v = drag_k = -1;
                break;
            case SDL_MOUSEMOTION:
                if (drag_v >= 0) {
                    float sens = (SDL_GetModState() & KMOD_SHIFT) ? 0.001f : 0.005f;
                    P[drag_v][drag_k] = clamp01(drag_n0 + (drag_y0 - (float)e.motion.y) * sens);
                }
                break;
            case SDL_MOUSEWHEEL: {
                int mx, my, v, k;
                SDL_GetMouseState(&mx, &my);
                if (hit_knob(mx, my, &v, &k)) {
                    float step = (SDL_GetModState() & KMOD_SHIFT) ? 0.005f : 0.02f;
                    P[v][k] = clamp01(P[v][k] + e.wheel.y * step);
                }
                break;
            }
            }
        }

        MidiMsg mm;
        while (ring_pop(&mm)) {
            int ch = (mm.st & 0x0F) + 1;
            if ((mm.st & 0xF0) == 0xB0) {
                printf("CC   ch=%-2d cc=%-3d val=%d\n", ch, mm.d1, mm.d2);
                snprintf(status, sizeof status, "CC %d  VAL %d  CH %d", mm.d1, mm.d2, ch);
            } else {
                printf("NOTE ch=%-2d note=%-3d vel=%d\n", ch, mm.d1, mm.d2);
                snprintf(status, sizeof status, "NOTE %d  VEL %d  CH %d", mm.d1, mm.d2, ch);
            }
            fflush(stdout);
        }
        int sv = atomic_load(&sel_voice);
        if (sv != prev_sel) {
            printf("--> CC knobs now control %s\n", ch_names[sv]);
            fflush(stdout);
            prev_sel = sv;
        }

        for (int i = 0; i < NV; i++) {
            if (atomic_exchange(&hit[i], 0)) flash[i] = 1.f;
            flash[i] *= 0.88f;
        }
        draw_ui(ren, flash, status);
        SDL_Delay(16);
    }

    jack_client_close(client);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
