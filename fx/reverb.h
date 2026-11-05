#ifndef REVERB_H
#define REVERB_H

#include "effect.h"

/* Stereo Freeverb-style reverb. Returns NULL on allocation failure. */
Effect *reverb_create(float sample_rate);

#endif
