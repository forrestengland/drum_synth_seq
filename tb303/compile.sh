gcc -O2 -Wall tb303.c -o tb303 $(sdl2-config --cflags --libs) -ljack -lm
# start jackd (or qjackctl) first, then:
./tb303
