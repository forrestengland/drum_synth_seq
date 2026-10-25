/*
 * main.c - JACK2 client: 2 in / 2 out, runs a serial rack of effects.
 *
 * The rack is built once before jack_activate() and never changes while the
 * audio thread is running, so process() needs no locking. (Adding/removing
 * effects live later will need an RT-safe swap, e.g. a pointer swap or a
 * lock-free command queue.)
 */
#include <jack/jack.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "delay.h"
#include "effect.h"
#include "ui.h"

#define MAX_FX 16

static jack_port_t *in_port[2], *out_port[2];
static Effect *rack[MAX_FX];
static int nfx;
static volatile sig_atomic_t g_quit;

/* Audio thread: no malloc, no locks, no printf. */
static int process(jack_nframes_t nframes, void *arg)
{
    (void)arg;
    float *in[2], *out[2];
    for (int c = 0; c < 2; c++) {
        in[c]  = jack_port_get_buffer(in_port[c], nframes);
        out[c] = jack_port_get_buffer(out_port[c], nframes);
        memcpy(out[c], in[c], nframes * sizeof(float));
    }
    for (int i = 0; i < nfx; i++)
        rack[i]->process(rack[i], out[0], out[1], nframes);
    return 0;
}

static void on_shutdown(void *arg)
{
    (void)arg;
    g_quit = 1;     /* JACK server went away; let the UI loop exit */
}

/* Connect system capture -> our inputs, our outputs -> system playback. */
static void autoconnect(jack_client_t *c)
{
    const char **cap = jack_get_ports(c, NULL, JACK_DEFAULT_AUDIO_TYPE,
                                      JackPortIsPhysical | JackPortIsOutput);
    const char **pb  = jack_get_ports(c, NULL, JACK_DEFAULT_AUDIO_TYPE,
                                      JackPortIsPhysical | JackPortIsInput);
    for (int i = 0; i < 2 && cap && cap[i]; i++)
        jack_connect(c, cap[i], jack_port_name(in_port[i]));
    for (int i = 0; i < 2 && pb && pb[i]; i++)
        jack_connect(c, jack_port_name(out_port[i]), pb[i]);
    if (cap) jack_free(cap);
    if (pb) jack_free(pb);
}

int main(int argc, char **argv)
{
    int autoconn = 1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n")) {
            autoconn = 0;
        } else {
            fprintf(stderr, "usage: %s [-n]\n  -n  don't auto-connect to system ports\n", argv[0]);
            return strcmp(argv[i], "-h") ? 1 : 0;
        }
    }

    jack_status_t status;
    jack_client_t *client = jack_client_open("fxrack", JackNoStartServer, &status);
    if (!client) {
        fprintf(stderr, "Could not connect to JACK (status 0x%x). Is jackd running?\n", status);
        return 1;
    }

    int rc = 1;
    float sr = (float)jack_get_sample_rate(client);

    /* build the rack (more effects later: just append here) */
    if (!(rack[nfx] = delay_create(sr))) {
        fprintf(stderr, "out of memory\n");
        goto done;
    }
    nfx++;

    const char *in_names[2]  = { "in_1", "in_2" };
    const char *out_names[2] = { "out_1", "out_2" };
    for (int c = 0; c < 2; c++) {
        in_port[c]  = jack_port_register(client, in_names[c],
                          JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
        out_port[c] = jack_port_register(client, out_names[c],
                          JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
        if (!in_port[c] || !out_port[c]) {
            fprintf(stderr, "could not register JACK ports\n");
            goto done;
        }
    }

    jack_set_process_callback(client, process, NULL);
    jack_on_shutdown(client, on_shutdown, NULL);
    if (jack_activate(client)) {
        fprintf(stderr, "could not activate JACK client\n");
        goto done;
    }
    if (autoconn)
        autoconnect(client);

    rc = ui_run(rack, nfx, &g_quit);

done:
    jack_client_close(client);      /* stops the audio thread before we free */
    for (int i = 0; i < nfx; i++)
        rack[i]->destroy(rack[i]);
    return rc;
}
