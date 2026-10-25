#ifndef UI_H
#define UI_H

#include <signal.h>
#include "effect.h"

/*
 * Opens the SDL2 window and runs the UI loop until the window is closed or
 * *quit becomes non-zero. The UI is generated from each effect's parameter
 * list, so new effects need no UI code.
 */
int ui_run(Effect **rack, int nfx, volatile sig_atomic_t *quit);

#endif
