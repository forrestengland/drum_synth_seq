/*
 * reverb.c - stereo reverb in the classic Freeverb/Schroeder-Moorer layout:
 * 8 parallel lowpass-feedback comb filters feeding 4 series allpass filters
 * per channel, with the right channel's delay lengths offset for stereo width.
 *
 * Controls: Room (decay length), Damp (high-frequency absorption),
 *           Width (stereo spread of the tail), Mix (% wet), Bypass.
 *
 * Parameters are smoothed per sample, and bypass is a short crossfade.
 * All buffers are allocated up front in reverb_create().
 */
#include "reverb.h"

#include <math.h>
#include <stdlib.h>

#define NCOMB 8
#define NALLPASS 4
#define STEREO_SPREAD 23        /* samples @ 44.1 kHz added to the right channel */
#define INPUT_GAIN 0.015f
#define WET_GAIN 3.0f
#define ALLPASS_FB 0.5f

enum { P_ROOM, P_DAMP, P_WIDTH, P_MIX };

/* Tunings from Freeverb, in samples at 44.1 kHz. */
static const int comb_tuning[NCOMB]    = { 1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617 };
static const int allpass_tuning[NALLPASS] = { 556, 441, 341, 225 };

typedef struct { float *buf; int len, idx; float store; } Comb;
typedef struct { float *buf; int len, idx; } Allpass;

typedef struct {
    Comb    comb[2][NCOMB];
    Allpass ap[2][NALLPASS];

    /* smoothed (current) values */
    float room, damp, width, mix, act;
    float c_smooth;
} Reverb;

static inline float flush_denormal(float x)
{
    return fabsf(x) < 1e-15f ? 0.0f : x;
}

static void reverb_process(Effect *fx, float *l, float *r, uint32_t n)
{
    Reverb *v = fx->state;
    float *io[2] = { l, r };

    const float room_t  = param_get(&fx->params[P_ROOM])  * 0.01f;
    const float damp_t  = param_get(&fx->params[P_DAMP])  * 0.01f;
    const float width_t = param_get(&fx->params[P_WIDTH]) * 0.01f;
    const float mix_t   = param_get(&fx->params[P_MIX])   * 0.01f;
    const float act_t   =
        atomic_load_explicit(&fx->bypass, memory_order_relaxed) ? 0.0f : 1.0f;

    float room = v->room, damp = v->damp, width = v->width;
    float mix = v->mix, act = v->act;
    const float c = v->c_smooth;

    for (uint32_t i = 0; i < n; i++) {
        room  += (room_t  - room)  * c;
        damp  += (damp_t  - damp)  * c;
        width += (width_t - width) * c;
        mix   += (mix_t   - mix)   * c;
        act   += (act_t   - act)   * c;

        const float feedback = 0.7f + 0.28f * room;     /* 0.70 .. 0.98 */
        const float d1 = damp * 0.4f;
        const float d2 = 1.0f - d1;

        const float in_l = io[0][i], in_r = io[1][i];
        const float in = (in_l + in_r) * INPUT_GAIN;    /* mono feed, as Freeverb */

        float wet[2];
        for (int ch = 0; ch < 2; ch++) {
            float acc = 0.0f;

            for (int k = 0; k < NCOMB; k++) {
                Comb *cb = &v->comb[ch][k];
                float out = cb->buf[cb->idx];
                cb->store = flush_denormal(out * d2 + cb->store * d1);
                cb->buf[cb->idx] = flush_denormal(in + cb->store * feedback);
                if (++cb->idx >= cb->len) cb->idx = 0;
                acc += out;
            }
            for (int k = 0; k < NALLPASS; k++) {
                Allpass *a = &v->ap[ch][k];
                float bo = a->buf[a->idx];
                float out = bo - acc;
                a->buf[a->idx] = flush_denormal(acc + bo * ALLPASS_FB);
                if (++a->idx >= a->len) a->idx = 0;
                acc = out;
            }
            wet[ch] = acc;
        }

        /* width: 1 = full stereo, 0 = mono tail */
        const float w1 = width * 0.5f + 0.5f;
        const float w2 = (1.0f - width) * 0.5f;
        const float wl = (wet[0] * w1 + wet[1] * w2) * WET_GAIN;
        const float wr = (wet[1] * w1 + wet[0] * w2) * WET_GAIN;

        const float out_l = in_l * (1.0f - mix) + wl * mix;
        const float out_r = in_r * (1.0f - mix) + wr * mix;
        io[0][i] = in_l + act * (out_l - in_l);         /* bypass crossfade */
        io[1][i] = in_r + act * (out_r - in_r);
    }

    v->room = room; v->damp = damp; v->width = width; v->mix = mix; v->act = act;
}

static void reverb_destroy(Effect *fx)
{
    if (!fx) return;
    Reverb *v = fx->state;
    if (v) {
        for (int ch = 0; ch < 2; ch++) {
            for (int k = 0; k < NCOMB; k++)    free(v->comb[ch][k].buf);
            for (int k = 0; k < NALLPASS; k++) free(v->ap[ch][k].buf);
        }
        free(v);
    }
    free(fx);
}

Effect *reverb_create(float sr)
{
    Effect *fx = calloc(1, sizeof *fx);
    Reverb *v  = calloc(1, sizeof *v);
    if (!fx || !v) { free(fx); free(v); return NULL; }

    fx->name    = "REVERB";
    fx->state   = v;
    fx->process = reverb_process;
    fx->destroy = reverb_destroy;
    fx->nparams = 4;
    atomic_init(&fx->bypass, 0);

    param_init(&fx->params[P_ROOM],  "ROOM",  "%.0f %%", 0.0f, 100.0f,  60.0f, 1.0f);
    param_init(&fx->params[P_DAMP],  "DAMP",  "%.0f %%", 0.0f, 100.0f,  40.0f, 1.0f);
    param_init(&fx->params[P_WIDTH], "WIDTH", "%.0f %%", 0.0f, 100.0f, 100.0f, 1.0f);
    param_init(&fx->params[P_MIX],   "MIX",   "%.0f %%", 0.0f, 100.0f,  25.0f, 1.0f);

    const float scale = sr / 44100.0f;
    for (int ch = 0; ch < 2; ch++) {
        int spread = ch ? (int)(STEREO_SPREAD * scale + 0.5f) : 0;
        for (int k = 0; k < NCOMB; k++) {
            Comb *cb = &v->comb[ch][k];
            cb->len = (int)(comb_tuning[k] * scale + 0.5f) + spread;
            cb->buf = calloc((size_t)cb->len, sizeof(float));
            if (!cb->buf) { reverb_destroy(fx); return NULL; }
        }
        for (int k = 0; k < NALLPASS; k++) {
            Allpass *a = &v->ap[ch][k];
            a->len = (int)(allpass_tuning[k] * scale + 0.5f) + spread;
            a->buf = calloc((size_t)a->len, sizeof(float));
            if (!a->buf) { reverb_destroy(fx); return NULL; }
        }
    }

    v->c_smooth = 1.0f - expf(-1.0f / (0.010f * sr));   /* ~10 ms */
    v->room  = param_get(&fx->params[P_ROOM])  * 0.01f;
    v->damp  = param_get(&fx->params[P_DAMP])  * 0.01f;
    v->width = param_get(&fx->params[P_WIDTH]) * 0.01f;
    v->mix   = param_get(&fx->params[P_MIX])   * 0.01f;
    v->act   = 1.0f;

    return fx;
}
