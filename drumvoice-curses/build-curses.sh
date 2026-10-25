#!/bin/sh
# Debian 11 deps: sudo apt install build-essential pkg-config libncurses-dev libjack-jackd2-dev
set -e
gcc -std=gnu11 -O2 -Wall -Wextra -o drumvoice-curses drumvoice-curses.c \
    $(pkg-config --cflags --libs jack ncurses) -lm
echo "built ./drumvoice-curses"
