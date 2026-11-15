#ifndef EQ3_H
#define EQ3_H

#include "effect.h"

/* DJ-style 3-band EQ with kill. Returns NULL on allocation failure. */
Effect *eq3_create(float sample_rate);

#endif
