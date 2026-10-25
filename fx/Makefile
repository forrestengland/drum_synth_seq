CC      ?= gcc
CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -Wall -Wextra

JACK_CFLAGS := $(shell pkg-config --cflags jack 2>/dev/null)
JACK_LIBS   := $(shell pkg-config --libs jack 2>/dev/null)
SDL_CFLAGS  := $(shell pkg-config --cflags sdl2 SDL2_ttf 2>/dev/null)
SDL_LIBS    := $(shell pkg-config --libs sdl2 SDL2_ttf 2>/dev/null)
NC_CFLAGS   := $(shell pkg-config --cflags ncurses 2>/dev/null)
NC_LIBS     := $(shell pkg-config --libs ncurses 2>/dev/null)

COMMON = main.o delay.o

all: fxrack fxrack-tui
sdl: fxrack
tui: fxrack-tui

fxrack: $(COMMON) ui.o
	$(CC) $(LDFLAGS) -o $@ $^ $(JACK_LIBS) $(SDL_LIBS) -lm

fxrack-tui: $(COMMON) tui.o
	$(CC) $(LDFLAGS) -o $@ $^ $(JACK_LIBS) $(NC_LIBS) -lm

main.o delay.o: CFLAGS += $(JACK_CFLAGS)
ui.o:  CFLAGS += $(SDL_CFLAGS)
tui.o: CFLAGS += $(NC_CFLAGS)

%.o: %.c effect.h delay.h ui.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f fxrack fxrack-tui *.o

.PHONY: all sdl tui clean
