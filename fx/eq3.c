/*
 * eq3.c - DJ-style 3-band EQ (isolator): LOW / MID / HIGH, each from full
 * kill up to +6 dB, with 0 dB at the knob's centre detent.
 *
 * The signal is split into three bands with Linkwitz-Riley (4th-order)
 * crossovers at 250 Hz and 2.5 kHz, each band is scaled, and they're summed.
 * LR4 splits sum flat, and the low band gets a matching all-pass at the upper
 * crossover so all three bands stay phase-aligned. With every knob at 0 dB
 * the output has the original magnitude response (phase-shifted, as any
 * crossover is).
 *
 *               +-- LP4(f1) -- allpass(f2) ------------------ x gain_low --+
 *   in --------+                                                           +--> out
 *               +-- HP4(f1) --+-- LP4(f2) -------------------- x gain_mid --+
 *                             +-- HP4(f2) -------------------- x gain_high-+
 *
 * An LR4 filter is two cascaded 2nd-order Butterworth biquads, and
 * LP4 + HP4 at the same frequency equals a 2nd-order Butterworth all-pass,
 * so the compensation is a single biquad.
 */
#include "eq3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define F_LOW_MID  250.0
#define F_MID_HIGH 2500.0

#define KILL_DB  -40.0f     /* knob minimum = kill (gain 0) */
#define MAX_DB     6.0f

enum { P_LOW, P_MID, P_HIGH };

typedef struct { float b0, b1, b2, a1, a2; } BiqCoef;
typedef struct { float z1, z2; } BiqState;

typedef struct {
    BiqState lp1[2], hp1[2];    /* LR4 at f1: low / rest */
    BiqState lp2[2], hp2[2];    /* LR4 at f2: mid / high, fed by hp1 */
    BiqState ap2;               /* all-pass at f2 on the low band */
} Chan;

typedef struct {
    Chan ch[2];
    BiqCoef lp1, hp1, lp2, hp2, ap2;
    float g_low, g_mid, g_high, act;    /* smoothed (current) values */
    float c_smooth;
} Eq3;

static inline float flush_denormal(float x)
{
    return fabsf(x) < 1e-15f ? 0.0f : x;
}

/* Transposed direct form II. */
static inline float biquad(const BiqCoef *c, BiqState *s, float x)
{
    float y = c->b0 * x + s->z1;
    s->z1 = flush_denormal(c->b1 * x - c->a1 * y + s->z2);
    s->z2 = flush_denormal(c->b2 * x - c->a2 * y);
    return y;
}

static inline float lr4(const BiqCoef *c, BiqState s[2], float x)
{
    return biquad(c, &s[1], biquad(c, &s[0], x));
}

/* RBJ cookbook, Butterworth Q. kind: 0 = low-pass, 1 = high-pass, 2 = all-pass */
static BiqCoef make_biquad(int kind, double f, double sr)
{
    const double pi = 3.14159265358979323846;
    const double q = 0.70710678118654752;
    double w0 = 2.0 * pi * f / sr;
    double cw = cos(w0), alpha = sin(w0) / (2.0 * q);
    double b0, b1, b2, a0 = 1.0 + alpha, a1 = -2.0 * cw, a2 = 1.0 - alpha;

    switch (kind) {
    case 0:  b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw;    b2 = b0;        break;
    case 1:  b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw); b2 = b0;        break;
    default: b0 = 1.0 - alpha;      b1 = -2.0 * cw;   b2 = 1.0 + alpha; break;
    }
    BiqCoef c = { (float)(b0 / a0), (float)(b1 / a0), (float)(b2 / a0),
                  (float)(a1 / a0), (float)(a2 / a0) };
    return c;
}

/* Knob dB -> linear gain; the bottom of the range is a true kill. */
static inline float knob_gain(float db)
{
    return db <= KILL_DB + 0.05f ? 0.0f : powf(10.0f, db * 0.05f);
}

static void eq3_process(Effect *fx, float *l, float *r, uint32_t n)
{
    Eq3 *e = fx->state;
    float *io[2] = { l, r };

    const float gl_t = knob_gain(param_get(&fx->params[P_LOW]));
    const float gm_t = knob_gain(param_get(&fx->params[P_MID]));
    const float gh_t = knob_gain(param_get(&fx->params[P_HIGH]));
    const float act_t =
        atomic_load_explicit(&fx->bypass, memory_order_relaxed) ? 0.0f : 1.0f;

    float gl = e->g_low, gm = e->g_mid, gh = e->g_high, act = e->act;
    const float cs = e->c_smooth;

    for (uint32_t i = 0; i < n; i++) {
        gl  += (gl_t  - gl)  * cs;
        gm  += (gm_t  - gm)  * cs;
        gh  += (gh_t  - gh)  * cs;
        act += (act_t - act) * cs;

        for (int c = 0; c < 2; c++) {
            Chan *s = &e->ch[c];
            const float x = io[c][i];

            const float low  = lr4(&e->lp1, s->lp1, x);
            const float rest = lr4(&e->hp1, s->hp1, x);
            const float mid  = lr4(&e->lp2, s->lp2, rest);
            const float high = lr4(&e->hp2, s->hp2, rest);
            const float low_aligned = biquad(&e->ap2, &s->ap2, low);

            const float y = low_aligned * gl + mid * gm + high * gh;
            io[c][i] = x + act * (y - x);               /* bypass crossfade */
        }
    }

    e->g_low = gl; e->g_mid = gm; e->g_high = gh; e->act = act;
}

static void eq3_destroy(Effect *fx)
{
    if (!fx) return;
    free(fx->state);
    free(fx);
}

/* Readout: "KILL" at the bottom, otherwise signed dB. */
static void eq3_format(float v, char *buf, size_t n)
{
    if (v <= KILL_DB + 0.05f) snprintf(buf, n, "KILL");
    else                      snprintf(buf, n, "%+.1f dB", v);
}

Effect *eq3_create(float sr)
{
    Effect *fx = calloc(1, sizeof *fx);
    Eq3    *e  = calloc(1, sizeof *e);
    if (!fx || !e) { free(fx); free(e); return NULL; }

    fx->name    = "DJ EQ";
    fx->state   = e;
    fx->process = eq3_process;
    fx->destroy = eq3_destroy;
    fx->nparams = 3;
    atomic_init(&fx->bypass, 0);

    /* Curve chosen so 0 dB lands at exactly half travel (centre detent):
     * KILL_DB + (MAX_DB - KILL_DB) * 0.5^c = 0  =>  c = ln(40/46) / ln(0.5).
     * That also makes the bottom of the knob drop quickly towards the kill. */
    const float curve = 0.20163f;
    const char *names[3] = { "LOW", "MID", "HIGH" };
    for (int i = 0; i < 3; i++) {
        param_init(&fx->params[i], names[i], "%+.1f dB", KILL_DB, MAX_DB, 0.0f, curve);
        fx->params[i].fmt_fn = eq3_format;
    }

    e->lp1 = make_biquad(0, F_LOW_MID,  sr);
    e->hp1 = make_biquad(1, F_LOW_MID,  sr);
    e->lp2 = make_biquad(0, F_MID_HIGH, sr);
    e->hp2 = make_biquad(1, F_MID_HIGH, sr);
    e->ap2 = make_biquad(2, F_MID_HIGH, sr);

    e->c_smooth = 1.0f - expf(-1.0f / (0.010f * sr));   /* ~10 ms */
    e->g_low = e->g_mid = e->g_high = 1.0f;
    e->act = 1.0f;

    return fx;
}
