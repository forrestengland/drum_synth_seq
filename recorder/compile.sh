gcc -O2 -Wall -o jackrec jackrec.c $(pkg-config --cflags --libs sdl2 SDL2_ttf jack) -lpthread
