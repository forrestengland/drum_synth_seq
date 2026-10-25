#ifndef DELAY_H
#define DELAY_H

#include "effect.h"

/* Stereo (dual-mono) feedback delay. Returns NULL on allocation failure. */
Effect *delay_create(float sample_rate);

#endif
