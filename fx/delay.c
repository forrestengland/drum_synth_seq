/*
 * delay.c - stereo feedback delay with interpolated, smoothed delay time.
 *
 * Controls: Time (ms), Feedback (%), Mix (% wet), Bypass.
 *
 * All parameter changes are smoothed per-sample (one-pole) so turning a knob
 * or hitting bypass doesn't click. Changing Time sweeps the read head, which
 * gives the classic tape-style pitch bend rather than a glitch.
 */
#include "delay.h"

#include <math.h>
#include <stdlib.h>

#define MAX_DELAY_SEC 2.0f

enum { P_TIME, P_FEEDBACK, P_MIX };

typedef struct {
    float *buf[2];      /* circular buffers, power-of-two length */
    uint32_t mask;      /* length - 1 */
    uint32_t w;         /* write index */
    float sr;
    float max_delay;    /* samples */

    /* smoothed (current) values */
    float time;         /* delay in samples */
    float fb;           /* 0..0.95 */
    float mix;          /* 0..1 */
    float act;          /* 1 = effect engaged, 0 = bypassed */

    float c_time, c_fast;   /* smoothing coefficients */
} Delay;

static inline float flush_denormal(float x)
{
    return fabsf(x) < 1e-15f ? 0.0f : x;
}

static void delay_process(Effect *fx, float *l, float *r, uint32_t n)
{
    Delay *d = fx->state;
    float *io[2] = { l, r };

    float t_target = param_get(&fx->params[P_TIME]) * 0.001f * d->sr;
    if (t_target < 1.0f) t_target = 1.0f;
    if (t_target > d->max_delay) t_target = d->max_delay;
    const float fb_target  = param_get(&fx->params[P_FEEDBACK]) * 0.01f;
    const float mix_target = param_get(&fx->params[P_MIX]) * 0.01f;
    const float act_target =
        atomic_load_explicit(&fx->bypass, memory_order_relaxed) ? 0.0f : 1.0f;

    const float len = (float)(d->mask + 1);
    uint32_t w = d->w;
    float time = d->time, fb = d->fb, mix = d->mix, act = d->act;

    for (uint32_t i = 0; i < n; i++) {
        time += (t_target   - time) * d->c_time;
        fb   += (fb_target  - fb)   * d->c_fast;
        mix  += (mix_target - mix)  * d->c_fast;
        act  += (act_target - act)  * d->c_fast;

        /* fractional read position behind the write head */
        float rpos = (float)w - time;
        if (rpos < 0.0f) rpos += len;
        uint32_t i0 = (uint32_t)rpos & d->mask;
        uint32_t i1 = (i0 + 1) & d->mask;
        float frac = rpos - floorf(rpos);

        for (int ch = 0; ch < 2; ch++) {
            float in  = io[ch][i];
            float del = d->buf[ch][i0] * (1.0f - frac) + d->buf[ch][i1] * frac;

            /* The line keeps running while bypassed so re-engaging is seamless. */
            d->buf[ch][w] = flush_denormal(in + del * fb);

            float out = in * (1.0f - mix) + del * mix;
            io[ch][i] = in + act * (out - in);   /* bypass crossfade */
        }
        w = (w + 1) & d->mask;
    }

    d->w = w;
    d->time = time; d->fb = fb; d->mix = mix; d->act = act;
}

static void delay_destroy(Effect *fx)
{
    if (!fx) return;
    Delay *d = fx->state;
    if (d) {
        free(d->buf[0]);
        free(d->buf[1]);
        free(d);
    }
    free(fx);
}

Effect *delay_create(float sr)
{
    Effect *fx = calloc(1, sizeof *fx);
    Delay  *d  = calloc(1, sizeof *d);
    if (!fx || !d) { free(fx); free(d); return NULL; }

    fx->name    = "DELAY";
    fx->state   = d;
    fx->process = delay_process;
    fx->destroy = delay_destroy;
    fx->nparams = 3;
    atomic_init(&fx->bypass, 0);

    param_init(&fx->params[P_TIME],     "TIME",     "%.0f ms", 1.0f, MAX_DELAY_SEC * 1000.0f, 350.0f, 2.0f);
    param_init(&fx->params[P_FEEDBACK], "FEEDBACK", "%.0f %%",  0.0f, 95.0f, 40.0f, 1.0f);
    param_init(&fx->params[P_MIX],      "MIX",      "%.0f %%",  0.0f, 100.0f, 35.0f, 1.0f);

    d->sr = sr;
    d->max_delay = MAX_DELAY_SEC * sr;
    uint32_t len = 1;
    while (len < (uint32_t)d->max_delay + 4) len <<= 1;
    d->mask = len - 1;
    d->buf[0] = calloc(len, sizeof(float));
    d->buf[1] = calloc(len, sizeof(float));
    if (!d->buf[0] || !d->buf[1]) { delay_destroy(fx); return NULL; }

    d->c_time = 1.0f - expf(-1.0f / (0.030f * sr));   /* ~30 ms glide */
    d->c_fast = 1.0f - expf(-1.0f / (0.010f * sr));   /* ~10 ms */

    /* start the smoothers at the defaults so there's no initial ramp */
    d->time = param_get(&fx->params[P_TIME]) * 0.001f * sr;
    d->fb   = param_get(&fx->params[P_FEEDBACK]) * 0.01f;
    d->mix  = param_get(&fx->params[P_MIX]) * 0.01f;
    d->act  = 1.0f;

    return fx;
}
