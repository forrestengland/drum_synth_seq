/*
 * jackrec - minimal JACK2 stereo audio recorder with an SDL2 GUI
 *
 * Build:
 *   gcc -O2 -Wall -o jackrec jackrec.c \
 *       $(pkg-config --cflags --libs sdl2 SDL2_ttf jack) -lpthread
 *
 * Debian/Ubuntu deps:
 *   sudo apt install libjack-jackd2-dev libsdl2-dev libsdl2-ttf-dev fonts-dejavu-core
 *
 * Usage:
 *   jackrec [-d output_dir] [-f font.ttf] [-n jack_client_name] [-a]
 *     -d  directory to record into (default: current working directory)
 *     -f  TTF font to use (default: search common system locations)
 *     -n  JACK client name (default: jackrec)
 *     -a  auto-connect inputs to the first two physical capture ports
 *
 * Output: stereo, 32-bit float WAV at the JACK server's sample rate.
 * Keys:   Enter = record, Esc = stop.
 */

#define _DEFAULT_SOURCE          /* usleep, realpath, strcasecmp */
#define _POSIX_C_SOURCE 200809L

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <jack/jack.h>
#include <jack/ringbuffer.h>

#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "jackrec assumes a little-endian host"
#endif

#define NCH            2
#define FRAME_BYTES    (NCH * sizeof(float))
#define HEADER_BYTES   58
#define MAX_DATA_BYTES 0xFFFF0000u   /* stay under the 4 GiB WAV limit */
#define RING_SECONDS   10
#define CHUNK_FRAMES   256
#define NAME_MAX_LEN   200

/* ------------------------------------------------------------------ */
/* Shared state                                                        */
/* ------------------------------------------------------------------ */

static jack_client_t    *client;
static jack_port_t      *in_port[NCH];
static jack_ringbuffer_t *ring;

static atomic_int       recording;      /* process callback writes to ring when 1 */
static atomic_int       disk_stop;      /* tells disk thread to drain and exit */
static atomic_int       server_down;
static atomic_int       rate_changed;
static atomic_int       limit_hit;
static atomic_int       write_error;
static atomic_ullong    frames_written;
static atomic_ullong    dropped_frames;

static FILE            *rec_fp;
static pthread_t        rec_thread;
static bool             rec_active;
static uint32_t         rec_rate;
static char             rec_path[PATH_MAX];

/* ------------------------------------------------------------------ */
/* WAV helpers                                                         */
/* ------------------------------------------------------------------ */

static void put16(unsigned char *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(unsigned char *p, uint32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

/* IEEE-float WAV: RIFF/WAVE, fmt (18 bytes), fact, data. */
static void write_header(FILE *f, uint32_t rate, uint64_t data_bytes)
{
    unsigned char h[HEADER_BYTES];
    memcpy(h + 0, "RIFF", 4);  put32(h + 4, (uint32_t)(50 + data_bytes));
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4); put32(h + 16, 18);
    put16(h + 20, 3);                       /* WAVE_FORMAT_IEEE_FLOAT */
    put16(h + 22, NCH);
    put32(h + 24, rate);
    put32(h + 28, rate * (uint32_t)FRAME_BYTES);
    put16(h + 32, (uint16_t)FRAME_BYTES);
    put16(h + 34, 32);
    put16(h + 36, 0);                       /* cbSize */
    memcpy(h + 38, "fact", 4); put32(h + 42, 4);
    put32(h + 46, (uint32_t)(data_bytes / FRAME_BYTES));
    memcpy(h + 50, "data", 4); put32(h + 54, (uint32_t)data_bytes);

    fseek(f, 0, SEEK_SET);
    fwrite(h, 1, sizeof h, f);
    fflush(f);
    fseek(f, 0, SEEK_END);
}

/* ------------------------------------------------------------------ */
/* JACK callbacks                                                      */
/* ------------------------------------------------------------------ */

static int process_cb(jack_nframes_t nframes, void *arg)
{
    (void)arg;
    if (!atomic_load(&recording))
        return 0;

    const float *in[NCH];
    for (int c = 0; c < NCH; c++)
        in[c] = jack_port_get_buffer(in_port[c], nframes);

    float tmp[NCH * CHUNK_FRAMES];
    jack_nframes_t done = 0;
    while (done < nframes) {
        jack_nframes_t n = nframes - done;
        if (n > CHUNK_FRAMES) n = CHUNK_FRAMES;
        for (jack_nframes_t i = 0; i < n; i++)
            for (int c = 0; c < NCH; c++)
                tmp[i * NCH + c] = in[c][done + i];

        size_t bytes = n * FRAME_BYTES;
        if (jack_ringbuffer_write_space(ring) >= bytes)
            jack_ringbuffer_write(ring, (const char *)tmp, bytes);
        else
            atomic_fetch_add(&dropped_frames, n);
        done += n;
    }
    return 0;
}

static int samplerate_cb(jack_nframes_t nframes, void *arg)
{
    (void)nframes; (void)arg;
    if (atomic_load(&recording))
        atomic_store(&rate_changed, 1);
    return 0;
}

static void shutdown_cb(void *arg)
{
    (void)arg;
    atomic_store(&server_down, 1);
}

/* ------------------------------------------------------------------ */
/* Disk thread                                                         */
/* ------------------------------------------------------------------ */

static void *disk_thread(void *arg)
{
    (void)arg;
    static char buf[1 << 16];
    uint64_t bytes = 0;
    struct timespec last_hdr, now;
    clock_gettime(CLOCK_MONOTONIC, &last_hdr);

    for (;;) {
        int stopping = atomic_load(&disk_stop);
        size_t avail = jack_ringbuffer_read_space(ring);
        avail -= avail % FRAME_BYTES;

        if (avail == 0) {
            if (stopping) break;
            usleep(5000);
            continue;
        }
        if (avail > sizeof buf) avail = sizeof buf;
        jack_ringbuffer_read(ring, buf, avail);

        if (bytes + avail > MAX_DATA_BYTES) {
            atomic_store(&limit_hit, 1);        /* discard, main thread will stop us */
            continue;
        }
        if (!atomic_load(&write_error)) {
            if (fwrite(buf, 1, avail, rec_fp) != avail) {
                atomic_store(&write_error, 1);
                continue;
            }
            bytes += avail;
            atomic_store(&frames_written, bytes / FRAME_BYTES);
        }

        /* refresh the header about once a second so a crash leaves a usable file */
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec != last_hdr.tv_sec) {
            write_header(rec_fp, rec_rate, bytes);
            last_hdr = now;
        }
    }

    write_header(rec_fp, rec_rate, bytes);
    fclose(rec_fp);
    rec_fp = NULL;
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Start / stop                                                        */
/* ------------------------------------------------------------------ */

static bool ends_with_wav(const char *s)
{
    size_t n = strlen(s);
    return n >= 4 && strcasecmp(s + n - 4, ".wav") == 0;
}

static bool start_recording(const char *dir, const char *input, char *err, size_t errsz)
{
    char base[NAME_MAX_LEN + 16];
    snprintf(base, sizeof base, "%s", input);

    /* trim whitespace */
    size_t len = strlen(base);
    while (len && (base[len - 1] == ' ' || base[len - 1] == '\t')) base[--len] = 0;
    char *b = base;
    while (*b == ' ' || *b == '\t') b++;
    if (b != base) memmove(base, b, strlen(b) + 1);

    if (ends_with_wav(base)) base[strlen(base) - 4] = 0;
    if (!base[0]) {
        time_t t = time(NULL);
        struct tm tm;
        localtime_r(&t, &tm);
        strftime(base, sizeof base, "rec_%Y%m%d_%H%M%S", &tm);
    }

    FILE *fp = NULL;
    char path[PATH_MAX];
    for (int n = 1; n < 10000; n++) {
        int r;
        if (n == 1) r = snprintf(path, sizeof path, "%s/%s.wav", dir, base);
        else        r = snprintf(path, sizeof path, "%s/%s_%d.wav", dir, base, n);
        if (r < 0 || (size_t)r >= sizeof path) {
            snprintf(err, errsz, "Path too long");
            return false;
        }
        fp = fopen(path, "wbx");           /* never overwrite an existing file */
        if (fp) break;
        if (errno != EEXIST) {
            snprintf(err, errsz, "Cannot create file: %s", strerror(errno));
            return false;
        }
    }
    if (!fp) {
        snprintf(err, errsz, "Could not find a free filename");
        return false;
    }

    rec_rate = jack_get_sample_rate(client);
    write_header(fp, rec_rate, 0);

    rec_fp = fp;
    snprintf(rec_path, sizeof rec_path, "%s", path);
    jack_ringbuffer_reset(ring);            /* safe: process callback is idle */
    atomic_store(&frames_written, 0);
    atomic_store(&dropped_frames, 0);
    atomic_store(&limit_hit, 0);
    atomic_store(&write_error, 0);
    atomic_store(&rate_changed, 0);
    atomic_store(&disk_stop, 0);

    if (pthread_create(&rec_thread, NULL, disk_thread, NULL) != 0) {
        fclose(fp);
        rec_fp = NULL;
        snprintf(err, errsz, "Could not start disk thread");
        return false;
    }
    rec_active = true;
    atomic_store(&recording, 1);
    return true;
}

static void stop_recording(void)
{
    if (!rec_active) return;
    atomic_store(&recording, 0);

    /* let any in-flight process() cycle finish before draining */
    jack_nframes_t bs = jack_get_buffer_size(client);
    useconds_t wait = (useconds_t)(2.0e6 * bs / (rec_rate ? rec_rate : 48000)) + 10000;
    usleep(wait);

    atomic_store(&disk_stop, 1);
    pthread_join(rec_thread, NULL);
    rec_active = false;
}

/* ------------------------------------------------------------------ */
/* GUI helpers                                                         */
/* ------------------------------------------------------------------ */

static const SDL_Color COL_BG    = { 30,  32,  36, 255};
static const SDL_Color COL_TEXT  = {230, 230, 230, 255};
static const SDL_Color COL_DIM   = {130, 130, 135, 255};
static const SDL_Color COL_BOX   = { 45,  48,  54, 255};
static const SDL_Color COL_BOXD  = { 38,  40,  44, 255};
static const SDL_Color COL_REC   = {200,  40,  40, 255};
static const SDL_Color COL_RECD  = { 90,  45,  45, 255};
static const SDL_Color COL_STOP  = { 70,  120, 190, 255};
static const SDL_Color COL_STOPD = { 45,  58,  78, 255};
static const SDL_Color COL_WARN  = {235, 170,  50, 255};

static void set_col(SDL_Renderer *r, SDL_Color c)
{
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

static void fill(SDL_Renderer *r, SDL_Rect rc, SDL_Color c)
{
    set_col(r, c);
    SDL_RenderFillRect(r, &rc);
}

static int draw_text(SDL_Renderer *r, TTF_Font *f, const char *s, int x, int y, SDL_Color c)
{
    if (!s || !*s) return 0;
    SDL_Surface *sf = TTF_RenderUTF8_Blended(f, s, c);
    if (!sf) return 0;
    SDL_Texture *t = SDL_CreateTextureFromSurface(r, sf);
    SDL_Rect d = {x, y, sf->w, sf->h};
    if (t) SDL_RenderCopy(r, t, NULL, &d);
    int w = sf->w;
    if (t) SDL_DestroyTexture(t);
    SDL_FreeSurface(sf);
    return w;
}

static void draw_text_centered(SDL_Renderer *r, TTF_Font *f, const char *s, SDL_Rect box, SDL_Color c)
{
    int w = 0, h = 0;
    TTF_SizeUTF8(f, s, &w, &h);
    draw_text(r, f, s, box.x + (box.w - w) / 2, box.y + (box.h - h) / 2, c);
}

static bool in_rect(SDL_Rect r, int x, int y)
{
    return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

static void utf8_pop(char *s)
{
    size_t n = strlen(s);
    if (!n) return;
    do { n--; } while (n > 0 && ((unsigned char)s[n] & 0xC0) == 0x80);
    s[n] = 0;
}

static void append_filtered(char *name, size_t cap, const char *txt)
{
    for (const char *p = txt; *p; p++) {
        if (*p == '/' || *p == '\\' || (unsigned char)*p < 32) continue;
        size_t n = strlen(name);
        if (n + 1 >= cap) break;
        name[n] = *p;
        name[n + 1] = 0;
    }
}

static void format_duration(char *out, size_t sz, double secs)
{
    long total = (long)secs;
    int tenths = (int)((secs - total) * 10.0);
    snprintf(out, sz, "%02ld:%02ld:%02ld.%d",
             total / 3600, (total / 60) % 60, total % 60, tenths);
}

static const char *find_font(const char *user)
{
    static const char *candidates[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
        "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
        "/usr/share/fonts/noto/NotoSansMono-Regular.ttf",
        "/System/Library/Fonts/Menlo.ttc",
        "/System/Library/Fonts/Supplemental/Courier New.ttf",
        "C:\\Windows\\Fonts\\consola.ttf",
        NULL
    };
    if (user) return user;
    for (int i = 0; candidates[i]; i++)
        if (access(candidates[i], R_OK) == 0) return candidates[i];
    return NULL;
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [-d output_dir] [-f font.ttf] [-n client_name] [-a]\n"
        "  -d  output directory (default: current directory)\n"
        "  -f  TTF font file\n"
        "  -n  JACK client name (default: jackrec)\n"
        "  -a  auto-connect to the first two physical capture ports\n", prog);
}

int main(int argc, char **argv)
{
    const char *dir = ".";
    const char *font_arg = NULL;
    const char *client_name = "jackrec";
    bool autoconnect = false;

    int opt;
    while ((opt = getopt(argc, argv, "d:f:n:ah")) != -1) {
        switch (opt) {
        case 'd': dir = optarg; break;
        case 'f': font_arg = optarg; break;
        case 'n': client_name = optarg; break;
        case 'a': autoconnect = true; break;
        default:  usage(argv[0]); return opt == 'h' ? 0 : 1;
        }
    }

    struct stat st;
    if (stat(dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "Output directory '%s' does not exist or is not a directory\n", dir);
        return 1;
    }
    char dir_abs[PATH_MAX];
    if (!realpath(dir, dir_abs)) snprintf(dir_abs, sizeof dir_abs, "%s", dir);

    /* ---- JACK ---- */
    jack_status_t jstatus;
    client = jack_client_open(client_name, JackNoStartServer, &jstatus);
    if (!client) {
        fprintf(stderr, "Could not connect to JACK (status 0x%x). Is the server running?\n", jstatus);
        return 1;
    }
    in_port[0] = jack_port_register(client, "in_L", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    in_port[1] = jack_port_register(client, "in_R", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    if (!in_port[0] || !in_port[1]) {
        fprintf(stderr, "Could not register JACK ports\n");
        jack_client_close(client);
        return 1;
    }

    size_t ring_bytes = (size_t)jack_get_sample_rate(client) * RING_SECONDS * FRAME_BYTES;
    ring = jack_ringbuffer_create(ring_bytes);
    if (!ring) {
        fprintf(stderr, "Could not allocate ringbuffer\n");
        jack_client_close(client);
        return 1;
    }
    jack_ringbuffer_mlock(ring);

    jack_set_process_callback(client, process_cb, NULL);
    jack_set_sample_rate_callback(client, samplerate_cb, NULL);
    jack_on_shutdown(client, shutdown_cb, NULL);

    if (jack_activate(client) != 0) {
        fprintf(stderr, "Could not activate JACK client\n");
        jack_ringbuffer_free(ring);
        jack_client_close(client);
        return 1;
    }

    if (autoconnect) {
        const char **ports = jack_get_ports(client, NULL, JACK_DEFAULT_AUDIO_TYPE,
                                            JackPortIsPhysical | JackPortIsOutput);
        if (ports) {
            for (int i = 0; i < NCH && ports[i]; i++)
                jack_connect(client, ports[i], jack_port_name(in_port[i]));
            jack_free(ports);
        }
    }

    /* ---- SDL ---- */
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    if (TTF_Init() != 0) {
        fprintf(stderr, "TTF_Init: %s\n", TTF_GetError());
        return 1;
    }
    const char *font_path = find_font(font_arg);
    if (!font_path) {
        fprintf(stderr, "No usable font found. Pass one with -f /path/to/font.ttf\n");
        return 1;
    }
    TTF_Font *font_s = TTF_OpenFont(font_path, 16);
    TTF_Font *font_m = TTF_OpenFont(font_path, 20);
    TTF_Font *font_l = TTF_OpenFont(font_path, 48);
    if (!font_s || !font_m || !font_l) {
        fprintf(stderr, "Could not open font '%s': %s\n", font_path, TTF_GetError());
        return 1;
    }

    const int W = 480, H = 280;
    SDL_Window *win = SDL_CreateWindow("JACK Recorder", SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED, W, H, 0);
    SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED |
                                                          SDL_RENDERER_PRESENTVSYNC) : NULL;
    if (win && !ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!win || !ren) {
        fprintf(stderr, "Window/renderer: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
    SDL_StartTextInput();

    const SDL_Rect box_name = { 20,  40, 440, 36};
    const SDL_Rect btn_rec  = { 20, 190, 210, 50};
    const SDL_Rect btn_stop = {250, 190, 210, 50};

    char name[NAME_MAX_LEN + 1] = "recording";
    char status[600] = "Ready. Connect sources to jackrec:in_L / in_R.";
    SDL_Color status_col = COL_DIM;
    double shown_secs = 0.0;
    bool running = true;

    while (running) {
        SDL_Event e;
        int have = SDL_WaitEventTimeout(&e, 50);
        while (have) {
            switch (e.type) {
            case SDL_QUIT:
                running = false;
                break;

            case SDL_TEXTINPUT:
                if (!rec_active) append_filtered(name, sizeof name, e.text.text);
                break;

            case SDL_KEYDOWN: {
                SDL_Keycode k = e.key.keysym.sym;
                if (k == SDLK_BACKSPACE && !rec_active) {
                    utf8_pop(name);
                } else if (k == SDLK_v && (e.key.keysym.mod & KMOD_CTRL) && !rec_active) {
                    char *clip = SDL_GetClipboardText();
                    if (clip) { append_filtered(name, sizeof name, clip); SDL_free(clip); }
                } else if ((k == SDLK_RETURN || k == SDLK_KP_ENTER) && !rec_active) {
                    goto do_record;
                } else if (k == SDLK_ESCAPE && rec_active) {
                    goto do_stop;
                }
                break;
            }

            case SDL_MOUSEBUTTONDOWN:
                if (e.button.button == SDL_BUTTON_LEFT) {
                    int mx = e.button.x, my = e.button.y;
                    if (in_rect(btn_rec, mx, my) && !rec_active) goto do_record;
                    if (in_rect(btn_stop, mx, my) && rec_active) goto do_stop;
                }
                break;
            }
            goto next_event;

        do_record: {
                char err[256];
                if (server_down) {
                    snprintf(status, sizeof status, "JACK server is gone - cannot record");
                    status_col = COL_WARN;
                } else if (start_recording(dir_abs, name, err, sizeof err)) {
                    shown_secs = 0.0;
                    snprintf(status, sizeof status, "Recording to %s", rec_path);
                    status_col = COL_TEXT;
                } else {
                    snprintf(status, sizeof status, "%s", err);
                    status_col = COL_WARN;
                }
            }
            goto next_event;

        do_stop:
            stop_recording();
            shown_secs = (double)atomic_load(&frames_written) / (rec_rate ? rec_rate : 1);
            {
                char d[32];
                format_duration(d, sizeof d, shown_secs);
                snprintf(status, sizeof status, "Saved %s (%s)", rec_path, d);
                status_col = COL_TEXT;
            }

        next_event:
            have = SDL_PollEvent(&e);
        }

        /* ---- conditions that force a stop ---- */
        if (rec_active) {
            const char *why = NULL;
            if (server_down)             why = "JACK server shut down - recording stopped";
            else if (rate_changed)       why = "JACK sample rate changed - recording stopped";
            else if (limit_hit)          why = "4 GB WAV size limit reached - recording stopped";
            else if (write_error)        why = "Disk write error - recording stopped";
            if (why) {
                stop_recording();
                shown_secs = (double)atomic_load(&frames_written) / (rec_rate ? rec_rate : 1);
                snprintf(status, sizeof status, "%s (%s)", why, rec_path);
                status_col = COL_WARN;
            }
        }
        if (rec_active) {
            shown_secs = (double)atomic_load(&frames_written) / (rec_rate ? rec_rate : 1);
            unsigned long long dr = atomic_load(&dropped_frames);
            if (dr) {
                snprintf(status, sizeof status,
                         "WARNING: dropped %llu frames (disk too slow)", dr);
                status_col = COL_WARN;
            }
        } else if (server_down && status_col.r != COL_WARN.r) {
            snprintf(status, sizeof status, "JACK server shut down");
            status_col = COL_WARN;
        }

        /* ---- draw ---- */
        set_col(ren, COL_BG);
        SDL_RenderClear(ren);

        draw_text(ren, font_s, "Filename (.wav added automatically)", 20, 14, COL_DIM);

        fill(ren, box_name, rec_active ? COL_BOXD : COL_BOX);
        {
            const char *vis = name;
            int w, h, maxw = box_name.w - 20;
            while (*vis) {
                TTF_SizeUTF8(font_m, vis, &w, &h);
                if (w <= maxw) break;
                vis++;
                while (((unsigned char)*vis & 0xC0) == 0x80) vis++;
            }
            int tx = box_name.x + 10, ty = box_name.y + 7;
            if (!*name) {
                draw_text(ren, font_m, "(empty = timestamp name)", tx, ty, COL_DIM);
            } else {
                int tw = draw_text(ren, font_m, vis, tx, ty, rec_active ? COL_DIM : COL_TEXT);
                if (!rec_active && (SDL_GetTicks() / 500) % 2 == 0) {
                    SDL_Rect cur = {tx + tw + 1, ty, 2, 22};
                    fill(ren, cur, COL_TEXT);
                }
            }
        }

        {
            char line[PATH_MAX + 16];
            snprintf(line, sizeof line, "Folder: %s", dir_abs);
            SDL_Rect clip = {20, 84, 440, 22};
            SDL_RenderSetClipRect(ren, &clip);
            draw_text(ren, font_s, line, 20, 84, COL_DIM);
            SDL_RenderSetClipRect(ren, NULL);
        }

        {
            char d[32];
            format_duration(d, sizeof d, shown_secs);
            SDL_Rect dbox = {20, 112, 440, 64};
            fill(ren, dbox, COL_BOXD);
            SDL_Color dc = rec_active ? (SDL_Color){255, 90, 90, 255} : COL_TEXT;
            draw_text_centered(ren, font_l, d, dbox, dc);

            if (rec_active && (SDL_GetTicks() / 500) % 2 == 0) {
                SDL_Rect dot = {dbox.x + 12, dbox.y + 12, 12, 12};
                fill(ren, dot, COL_REC);
            }
            char info[64];
            snprintf(info, sizeof info, "%u Hz  stereo  32-bit float",
                     (unsigned)(rec_active ? rec_rate : jack_get_sample_rate(client)));
            int iw = 0, ih = 0;
            TTF_SizeUTF8(font_s, info, &iw, &ih);
            draw_text(ren, font_s, info, dbox.x + dbox.w - iw - 8, dbox.y + dbox.h - ih - 4, COL_DIM);
        }

        fill(ren, btn_rec, rec_active ? COL_RECD : COL_REC);
        draw_text_centered(ren, font_m, "RECORD", btn_rec, rec_active ? COL_DIM : COL_TEXT);
        fill(ren, btn_stop, rec_active ? COL_STOP : COL_STOPD);
        draw_text_centered(ren, font_m, "STOP", btn_stop, rec_active ? COL_TEXT : COL_DIM);

        {
            SDL_Rect clip = {20, 252, 440, 22};
            SDL_RenderSetClipRect(ren, &clip);
            draw_text(ren, font_s, status, 20, 252, status_col);
            SDL_RenderSetClipRect(ren, NULL);
        }

        SDL_RenderPresent(ren);
    }

    /* ---- cleanup ---- */
    stop_recording();
    jack_deactivate(client);
    jack_client_close(client);
    jack_ringbuffer_free(ring);

    TTF_CloseFont(font_s);
    TTF_CloseFont(font_m);
    TTF_CloseFont(font_l);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    TTF_Quit();
    SDL_Quit();
    return 0;
}
