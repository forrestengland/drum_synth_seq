gcc -O2 -Wall -o drumseq drumseq.c $(pkg-config --cflags --libs sdl2 jack) -lm
#./drumseq drumsynth:midi_in      # optional: auto-connect to your synth's MIDI input port
#./drumseq drumsynth:midi_in 10   # optional: MIDI channel 1-16 (default 1)
