
gcc -O2 -Wall -o jackrec-tui jackrec_tui.c $(pkg-config --cflags --libs ncursesw jack) -lpthread
