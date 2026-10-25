/*
 * effect.h - the interface every effect in the rack implements.
 *
 * Threading model:
 *   - The JACK audio thread calls fx->process() and only READS params/bypass.
 *   - The UI thread WRITES params/bypass.
 *   - Everything shared is a lock-free atomic, so the audio thread never
 *     blocks, allocates, or takes a mutex.
 */
#ifndef EFFECT_H
#define EFFECT_H

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>

#define MAX_PARAMS 8

typedef struct {
    const char *name;   /* label under the knob                       */
    const char *fmt;    /* printf format for the readout, e.g. "%.0f ms" */
    float min, max, def;
    float curve;        /* 1 = linear, 2 = finer resolution near min  */
    _Atomic float value;
} Param;

typedef struct Effect {
    const char *name;
    Param params[MAX_PARAMS];
    int nparams;
    atomic_bool bypass;

    /* Process n frames in place on the left/right buffers. Audio thread. */
    void (*process)(struct Effect *fx, float *l, float *r, uint32_t n);
    void (*destroy)(struct Effect *fx);
    void *state;
} Effect;

static inline void param_init(Param *p, const char *name, const char *fmt,
                              float min, float max, float def, float curve)
{
    p->name = name;
    p->fmt = fmt;
    p->min = min;
    p->max = max;
    p->def = def;
    p->curve = curve;
    atomic_init(&p->value, def);
}

static inline float param_get(Param *p)
{
    return atomic_load_explicit(&p->value, memory_order_relaxed);
}

static inline void param_set(Param *p, float v)
{
    if (v < p->min) v = p->min;
    if (v > p->max) v = p->max;
    atomic_store_explicit(&p->value, v, memory_order_relaxed);
}

/* Knob position (0..1) <-> parameter value, honouring the curve. */
static inline float param_to_norm(Param *p, float v)
{
    float n = (v - p->min) / (p->max - p->min);
    if (n < 0.f) n = 0.f;
    if (n > 1.f) n = 1.f;
    return powf(n, 1.0f / p->curve);
}

static inline float param_from_norm(Param *p, float n)
{
    if (n < 0.f) n = 0.f;
    if (n > 1.f) n = 1.f;
    return p->min + (p->max - p->min) * powf(n, p->curve);
}

#endif
