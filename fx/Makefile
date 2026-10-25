CC      ?= gcc
CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -Wall -Wextra $(shell pkg-config --cflags jack sdl2 SDL2_ttf)
LDLIBS  += $(shell pkg-config --libs jack sdl2 SDL2_ttf) -lm

OBJ = main.o ui.o delay.o

fxrack: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

$(OBJ): effect.h delay.h ui.h

clean:
	rm -f fxrack $(OBJ)

.PHONY: clean
