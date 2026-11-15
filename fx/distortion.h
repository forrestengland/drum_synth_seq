#ifndef DISTORTION_H
#define DISTORTION_H

#include "effect.h"

/* Oversampled asymmetric-tanh distortion. Returns NULL on allocation failure. */
Effect *distortion_create(float sample_rate);

#endif
