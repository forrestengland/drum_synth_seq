#!/bin/sh
# Debian 11 deps: sudo apt install build-essential pkg-config libsdl2-dev libjack-jackd2-dev
set -e
gcc -std=gnu11 -O2 -Wall -Wextra -o drumvoice drumvoice.c \
    $(pkg-config --cflags --libs sdl2 jack) -lm
echo "built ./drumvoice"
