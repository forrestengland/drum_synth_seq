/*
 * distortion.c - warm saturation aimed at analog synth (saws, squares, bass).
 *
 * Signal path per channel:
 *
 *   in -> 2x upsample -> gain -> tanh(x + bias) - tanh(bias) -> 2x downsample
 *      -> DC blocker -> tone (12 dB/oct low-pass) -> level -> dry/wet mix
 *
 * Why it sounds decent on synths:
 *   - tanh is a smooth saturator: it thickens and rounds a saw rather than
 *     making fizzy hard clipping.
 *   - BIAS shifts the signal before the shaper so the clipping is lopsided.
 *     That adds even harmonics (octave-up body, a tube/transistor flavour)
 *     on top of the odd ones symmetric clipping makes. The static offset is
 *     subtracted and a DC blocker removes what's left.
 *   - 2x oversampling with a 31-tap half-band filter keeps the harmonics
 *     that would land above Nyquist from folding back as inharmonic aliasing,
 *     which is very audible on bright, high-pitched synth notes.
 *   - TONE rolls off the top end after the shaper, like the tone control on
 *     a drive pedal.
 *
 * Latency is 15 samples (the two half-band filters). The dry path in the
 * MIX control is delayed to match, so parallel mixing doesn't comb-filter.
 *
 * Controls: Drive (dB), Bias (%), Tone (Hz), Level (dB), Mix (%), Bypass.
 */
#include "distortion.h"

#include <math.h>
#include <stdlib.h>

#define HB_TAPS   16        /* non-zero half-band taps (31-tap filter overall) */
#define LATENCY   15        /* samples at the base rate */
#define MAX_BIAS  0.8f      /* bias at 100%, in shaper-input units */

enum { P_DRIVE, P_BIAS, P_TONE, P_LEVEL, P_MIX };

typedef struct {
    float xh[16];       /* input history for the upsampler */
    float vh[32];       /* 2x-rate history for the downsampler */
    float dry[16];      /* delay line to time-align the dry signal */
    uint32_t xi, vi, di;
    float dc_x1, dc_y1; /* DC blocker */
    float lp1, lp2;     /* tone filter */
} Chan;

typedef struct {
    Chan ch[2];
    float hb[HB_TAPS];  /* odd-offset half-band taps, normalised to sum 0.5 */
    float sr;
    float dc_r;

    /* smoothed (current) values */
    float gain, bias, tone_a, level, mix, act;
    float c_smooth;
} Dist;

static inline float flush_denormal(float x)
{
    return fabsf(x) < 1e-15f ? 0.0f : x;
}

static void dist_process(Effect *fx, float *l, float *r, uint32_t n)
{
    Dist *d = fx->state;
    float *io[2] = { l, r };

    const float gain_t  = powf(10.0f, param_get(&fx->params[P_DRIVE]) * 0.05f);
    const float bias_t  = param_get(&fx->params[P_BIAS]) * 0.01f * MAX_BIAS;
    const float tone_hz = param_get(&fx->params[P_TONE]);
    const float tone_t  = 1.0f - expf(-6.2831853f * tone_hz / d->sr);
    const float level_t = powf(10.0f, param_get(&fx->params[P_LEVEL]) * 0.05f);
    const float mix_t   = param_get(&fx->params[P_MIX]) * 0.01f;
    const float act_t   =
        atomic_load_explicit(&fx->bypass, memory_order_relaxed) ? 0.0f : 1.0f;

    float gain = d->gain, bias = d->bias, tone_a = d->tone_a;
    float level = d->level, mix = d->mix, act = d->act;
    const float cs = d->c_smooth;
    const float dc_r = d->dc_r;

    for (uint32_t i = 0; i < n; i++) {
        gain   += (gain_t  - gain)   * cs;
        bias   += (bias_t  - bias)   * cs;
        tone_a += (tone_t  - tone_a) * cs;
        level  += (level_t - level)  * cs;
        mix    += (mix_t   - mix)    * cs;
        act    += (act_t   - act)    * cs;
        const float bias_dc = tanhf(bias);

        for (int c = 0; c < 2; c++) {
            Chan *s = &d->ch[c];
            const float x = io[c][i];

            /* time-aligned dry copy */
            s->dry[s->di] = x;
            const float dry = s->dry[(s->di + 16 - LATENCY) & 15];
            s->di = (s->di + 1) & 15;

            /* 2x upsample (polyphase half-band): one phase is the filtered
             * in-between sample, the other is just the delayed input. */
            s->xh[s->xi] = x;
            float ye = 0.0f;
            for (int k = 0; k < HB_TAPS; k++)
                ye += d->hb[k] * s->xh[(s->xi - k) & 15];
            ye *= 2.0f;
            const float yo = s->xh[(s->xi - 7) & 15];
            s->xi = (s->xi + 1) & 15;

            /* waveshaper at 2x rate */
            const float a = tanhf(ye * gain + bias) - bias_dc;
            const float b = tanhf(yo * gain + bias) - bias_dc;

            /* 2x downsample: filter, keep one sample per pair */
            s->vh[s->vi] = a;
            float y = 0.5f * s->vh[(s->vi - 15) & 31];
            for (int k = 0; k < HB_TAPS; k++)
                y += d->hb[k] * s->vh[(s->vi - 2 * k) & 31];
            s->vi = (s->vi + 1) & 31;
            s->vh[s->vi] = b;
            s->vi = (s->vi + 1) & 31;

            /* DC blocker (bias leaves some DC on dynamic material) */
            const float dc = y - s->dc_x1 + dc_r * s->dc_y1;
            s->dc_x1 = y;
            s->dc_y1 = flush_denormal(dc);

            /* tone: two cascaded one-poles = 12 dB/oct, no resonance */
            s->lp1 += tone_a * (s->dc_y1 - s->lp1);
            s->lp2 += tone_a * (s->lp1 - s->lp2);
            s->lp1 = flush_denormal(s->lp1);
            s->lp2 = flush_denormal(s->lp2);

            const float wet = s->lp2 * level;
            const float out = dry * (1.0f - mix) + wet * mix;
            io[c][i] = x + act * (out - x);             /* bypass crossfade */
        }
    }

    d->gain = gain; d->bias = bias; d->tone_a = tone_a;
    d->level = level; d->mix = mix; d->act = act;
}

static void dist_destroy(Effect *fx)
{
    if (!fx) return;
    free(fx->state);
    free(fx);
}

Effect *distortion_create(float sr)
{
    Effect *fx = calloc(1, sizeof *fx);
    Dist   *d  = calloc(1, sizeof *d);
    if (!fx || !d) { free(fx); free(d); return NULL; }

    fx->name    = "DISTORTION";
    fx->state   = d;
    fx->process = dist_process;
    fx->destroy = dist_destroy;
    fx->nparams = 5;
    atomic_init(&fx->bypass, 0);

    param_init(&fx->params[P_DRIVE], "DRIVE", "+%.0f dB",   0.0f,    40.0f,   12.0f, 1.0f);
    param_init(&fx->params[P_BIAS],  "BIAS",  "%.0f %%",    0.0f,   100.0f,   25.0f, 1.0f);
    param_init(&fx->params[P_TONE],  "TONE",  "%.0f Hz",  500.0f, 16000.0f, 6000.0f, 2.0f);
    param_init(&fx->params[P_LEVEL], "LEVEL", "%.1f dB", -30.0f,     6.0f,   -6.0f, 1.0f);
    param_init(&fx->params[P_MIX],   "MIX",   "%.0f %%",    0.0f,   100.0f,  100.0f, 1.0f);

    /* Half-band low-pass (cutoff fs/4 of the 2x rate), Blackman-windowed.
     * Only the odd-offset taps are non-zero; the centre tap is 0.5. */
    const double pi = 3.14159265358979323846;
    double sum = 0.0, tmp[HB_TAPS];
    for (int i = 0; i < HB_TAPS; i++) {
        double k = 2.0 * i - 15.0;                      /* -15 .. +15, odd */
        double h = sin(pi * k / 2.0) / (pi * k);
        double w = 0.42 + 0.5 * cos(pi * k / 16.0) + 0.08 * cos(2.0 * pi * k / 16.0);
        tmp[i] = h * w;
        sum += tmp[i];
    }
    for (int i = 0; i < HB_TAPS; i++)
        d->hb[i] = (float)(tmp[i] * 0.5 / sum);         /* unity DC gain */

    d->sr = sr;
    d->dc_r = 1.0f - 6.2831853f * 15.0f / sr;           /* ~15 Hz high-pass */
    d->c_smooth = 1.0f - expf(-1.0f / (0.010f * sr));   /* ~10 ms */

    d->gain   = powf(10.0f, param_get(&fx->params[P_DRIVE]) * 0.05f);
    d->bias   = param_get(&fx->params[P_BIAS]) * 0.01f * MAX_BIAS;
    d->tone_a = 1.0f - expf(-6.2831853f * param_get(&fx->params[P_TONE]) / sr);
    d->level  = powf(10.0f, param_get(&fx->params[P_LEVEL]) * 0.05f);
    d->mix    = param_get(&fx->params[P_MIX]) * 0.01f;
    d->act    = 1.0f;

    return fx;
}
