/*
 * jackrec_tui - minimal JACK2 stereo audio recorder with an ncurses TUI
 *
 * Build:
 *   gcc -O2 -Wall -o jackrec-tui jackrec_tui.c \
 *       $(pkg-config --cflags --libs ncursesw jack) -lpthread
 *
 * Debian/Ubuntu deps:
 *   sudo apt install libjack-jackd2-dev libncurses-dev
 *
 * Usage:
 *   jackrec-tui [-d output_dir] [-n jack_client_name] [-a]
 *     -d  directory to record into (default: current working directory)
 *     -n  JACK client name (default: jackrec)
 *     -a  auto-connect inputs to the first two physical capture ports
 *
 * Output: stereo, 32-bit float WAV at the JACK server's sample rate.
 *
 * Keys:
 *   type        edit filename (when not recording)
 *   Backspace   delete character          Ctrl-U  clear filename
 *   Enter       start recording           Esc     stop recording
 *   F10/Ctrl-C  stop (if recording) and quit
 *   Mouse       click RECORD / STOP buttons
 */

#define _DEFAULT_SOURCE          /* usleep, realpath, strcasecmp */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
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
#include <locale.h>

#include <jack/jack.h>
#include <jack/ringbuffer.h>
#include <ncurses.h>

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

static jack_client_t     *client;
static jack_port_t       *in_port[NCH];
static jack_ringbuffer_t *ring;

static atomic_int    recording;      /* process callback writes to ring when 1 */
static atomic_int    disk_stop;      /* tells disk thread to drain and exit */
static atomic_int    server_down;
static atomic_int    rate_changed;
static atomic_int    limit_hit;
static atomic_int    write_error;
static atomic_ullong frames_written;
static atomic_ullong dropped_frames;

static FILE       *rec_fp;
static pthread_t   rec_thread;
static bool        rec_active;
static uint32_t    rec_rate;
static char        rec_path[PATH_MAX];

static volatile sig_atomic_t quit_sig;

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
            atomic_store(&limit_hit, 1);        /* discard; main thread will stop us */
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
/* UI state & actions                                                  */
/* ------------------------------------------------------------------ */

typedef struct { int y, x, h, w; } Rect;

enum { CP_REC = 1, CP_STOP, CP_WARN, CP_RECTXT };

static char   name[NAME_MAX_LEN + 1] = "recording";
static char   status[PATH_MAX + 128] = "Ready. Connect sources to in_L / in_R.";
static bool   status_warn;
static double shown_secs;
static char   dir_abs[PATH_MAX];
static const char *client_name_g = "jackrec";
static Rect   btn_rec, btn_stop;

static void set_status(bool warn, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void set_status(bool warn, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(status, sizeof status, fmt, ap);
    va_end(ap);
    status_warn = warn;
}

static void format_duration(char *out, size_t sz, double secs)
{
    long total = (long)secs;
    int tenths = (int)((secs - total) * 10.0);
    snprintf(out, sz, "%02ld:%02ld:%02ld.%d",
             total / 3600, (total / 60) % 60, total % 60, tenths);
}

static double current_secs(void)
{
    return (double)atomic_load(&frames_written) / (rec_rate ? rec_rate : 1);
}

static void do_record(void)
{
    if (rec_active) return;
    char err[256];
    if (atomic_load(&server_down)) {
        set_status(true, "JACK server is gone - cannot record");
    } else if (start_recording(dir_abs, name, err, sizeof err)) {
        shown_secs = 0.0;
        set_status(false, "Recording to %s", rec_path);
    } else {
        set_status(true, "%s", err);
    }
}

static void do_stop(void)
{
    if (!rec_active) return;
    stop_recording();
    shown_secs = current_secs();
    char d[32];
    format_duration(d, sizeof d, shown_secs);
    set_status(false, "Saved %s (%s)", rec_path, d);
}

/* ---- filename editing ---- */

static void utf8_pop(char *s)
{
    size_t n = strlen(s);
    if (!n) return;
    do { n--; } while (n > 0 && ((unsigned char)s[n] & 0xC0) == 0x80);
    s[n] = 0;
}

static int utf8_cols(const char *s)
{
    int n = 0;
    for (; *s; s++)
        if (((unsigned char)*s & 0xC0) != 0x80) n++;
    return n;
}

static void name_append_byte(unsigned char c)
{
    if (c == '/' || c == '\\' || c < 32 || c == 127) return;
    size_t n = strlen(name);
    if (n + 1 >= sizeof name) return;
    name[n] = (char)c;
    name[n + 1] = 0;
}

/* ------------------------------------------------------------------ */
/* Drawing                                                             */
/* ------------------------------------------------------------------ */

static void draw_frame(int y, int x, int h, int w)
{
    mvaddch(y, x, ACS_ULCORNER);
    mvhline(y, x + 1, ACS_HLINE, w - 2);
    mvaddch(y, x + w - 1, ACS_URCORNER);
    for (int i = 1; i < h - 1; i++) {
        mvaddch(y + i, x, ACS_VLINE);
        mvaddch(y + i, x + w - 1, ACS_VLINE);
    }
    mvaddch(y + h - 1, x, ACS_LLCORNER);
    mvhline(y + h - 1, x + 1, ACS_HLINE, w - 2);
    mvaddch(y + h - 1, x + w - 1, ACS_LRCORNER);
}

static void draw_centered(int y, int x, int w, const char *s, int cols)
{
    int pad = (w - cols) / 2;
    if (pad < 0) pad = 0;
    mvaddstr(y, x + pad, s);
}

static void draw_button(Rect r, const char *label, bool enabled, int pair)
{
    int attr = enabled ? (has_colors() ? (COLOR_PAIR(pair) | A_BOLD) : A_REVERSE)
                       : A_DIM;
    attron(attr);
    for (int i = 0; i < r.h; i++)
        mvhline(r.y + i, r.x, ' ', r.w);
    int len = (int)strlen(label);
    mvaddstr(r.y + r.h / 2, r.x + (r.w - len) / 2, label);
    attroff(attr);
    if (!enabled) {
        /* outline so it still reads as a button */
        attron(A_DIM);
        mvaddch(r.y, r.x, '['); mvaddch(r.y, r.x + r.w - 1, ']');
        attroff(A_DIM);
    }
}

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void draw(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    erase();

    if (rows < 17 || cols < 44) {
        mvprintw(0, 0, "Terminal too small");
        mvprintw(1, 0, "need 44x17, have %dx%d", cols, rows);
        curs_set(0);
        refresh();
        return;
    }

    const int x0 = 2;
    int ow = cols - 4;
    if (ow > 66) ow = 66;
    int inner = ow - 2;
    bool blink_on = (now_ms() / 500) % 2 == 0;

    /* title bar */
    attron(A_REVERSE);
    mvhline(0, 0, ' ', cols);
    mvprintw(0, 1, "JACK Recorder");
    {
        char right[96];
        snprintf(right, sizeof right, "client: %s", client_name_g);
        int rl = (int)strlen(right);
        if (cols > rl + 20) mvaddstr(0, cols - rl - 1, right);
    }
    attroff(A_REVERSE);

    /* filename field */
    mvprintw(2, x0, "Filename (.wav is added automatically):");

    const char *vis = name;
    while (*vis && utf8_cols(vis) > inner - 2) {
        vis++;
        while (((unsigned char)*vis & 0xC0) == 0x80) vis++;
    }
    mvaddch(3, x0, '[');
    mvaddch(3, x0 + ow - 1, ']');
    int fattr = A_UNDERLINE | (rec_active ? A_DIM : 0);
    attron(fattr);
    mvhline(3, x0 + 1, ' ', inner);
    if (*name) {
        mvaddstr(3, x0 + 2, vis);
    }
    attroff(fattr);
    if (!*name) {
        attron(A_DIM);
        mvaddstr(3, x0 + 2, "(empty = timestamp name)");
        attroff(A_DIM);
    }

    attron(A_DIM);
    mvprintw(4, x0, "Folder: %.*s", cols - x0 - 9, dir_abs);
    attroff(A_DIM);

    /* duration box */
    draw_frame(6, x0, 4, ow);
    {
        char d[32];
        format_duration(d, sizeof d, shown_secs);
        int attr = A_BOLD;
        if (rec_active && has_colors()) attr |= COLOR_PAIR(CP_RECTXT);
        attron(attr);
        draw_centered(7, x0 + 1, inner, d, (int)strlen(d));
        attroff(attr);

        if (rec_active && blink_on) {
            attron(COLOR_PAIR(CP_RECTXT) | A_BOLD);
            mvaddstr(7, x0 + 3, "\xE2\x97\x8F REC");   /* ● REC */
            attroff(COLOR_PAIR(CP_RECTXT) | A_BOLD);
        }

        char info[64];
        snprintf(info, sizeof info, "%u Hz | stereo | 32-bit float",
                 (unsigned)(rec_active ? rec_rate : jack_get_sample_rate(client)));
        attron(A_DIM);
        draw_centered(8, x0 + 1, inner, info, (int)strlen(info));
        attroff(A_DIM);
    }

    /* buttons */
    int bw = (ow - 2) / 2;
    btn_rec  = (Rect){11, x0,           3, bw};
    btn_stop = (Rect){11, x0 + bw + 2,  3, bw};
    draw_button(btn_rec,  "RECORD (Enter)", !rec_active, CP_REC);
    draw_button(btn_stop, "STOP (Esc)",      rec_active, CP_STOP);

    /* status */
    {
        int attr = status_warn ? (has_colors() ? COLOR_PAIR(CP_WARN) | A_BOLD : A_BOLD) : A_NORMAL;
        attron(attr);
        mvprintw(15, x0, "%.*s", cols - x0 - 1, status);
        attroff(attr);
    }

    /* footer */
    attron(A_DIM);
    mvprintw(rows - 1, x0, "%.*s", cols - x0 - 1,
             "Enter record  Esc stop  ^U clear name  F10/^C quit");
    attroff(A_DIM);

    /* cursor in the filename field */
    if (!rec_active) {
        curs_set(1);
        move(3, x0 + 2 + (*name ? utf8_cols(vis) : 0));
    } else {
        curs_set(0);
    }
    refresh();
}

/* ------------------------------------------------------------------ */
/* Misc                                                                */
/* ------------------------------------------------------------------ */

static void on_signal(int sig) { (void)sig; quit_sig = 1; }

static bool hit(Rect r, int y, int x)
{
    return y >= r.y && y < r.y + r.h && x >= r.x && x < r.x + r.w;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [-d output_dir] [-n client_name] [-a]\n"
        "  -d  output directory (default: current directory)\n"
        "  -n  JACK client name (default: jackrec)\n"
        "  -a  auto-connect to the first two physical capture ports\n", prog);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *dir = ".";
    bool autoconnect = false;

    int opt;
    while ((opt = getopt(argc, argv, "d:n:ah")) != -1) {
        switch (opt) {
        case 'd': dir = optarg; break;
        case 'n': client_name_g = optarg; break;
        case 'a': autoconnect = true; break;
        default:  usage(argv[0]); return opt == 'h' ? 0 : 1;
        }
    }

    struct stat st;
    if (stat(dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "Output directory '%s' does not exist or is not a directory\n", dir);
        return 1;
    }
    if (!realpath(dir, dir_abs)) snprintf(dir_abs, sizeof dir_abs, "%s", dir);

    /* ---- JACK ---- */
    jack_status_t jstatus;
    client = jack_client_open(client_name_g, JackNoStartServer, &jstatus);
    if (!client) {
        fprintf(stderr, "Could not connect to JACK (status 0x%x). Is the server running?\n", jstatus);
        return 1;
    }
    client_name_g = jack_get_client_name(client);   /* may differ if name was taken */

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

    /* ---- signals ---- */
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);

    /* ---- ncurses ---- */
    setlocale(LC_ALL, "");
    initscr();
    raw();                    /* Ctrl-C arrives as a key (3), no flow control */
    noecho();
    keypad(stdscr, TRUE);
    set_escdelay(25);         /* make Esc responsive */
    timeout(100);             /* getch() returns ERR after 100 ms -> 10 Hz refresh */
    mousemask(BUTTON1_CLICKED | BUTTON1_PRESSED, NULL);
    mouseinterval(0);
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(CP_REC,    COLOR_WHITE,  COLOR_RED);
        init_pair(CP_STOP,   COLOR_WHITE,  COLOR_BLUE);
        init_pair(CP_WARN,   COLOR_YELLOW, -1);
        init_pair(CP_RECTXT, COLOR_RED,    -1);
    }

    bool running = true;
    bool down_reported = false;

    while (running && !quit_sig) {
        int ch = getch();
        switch (ch) {
        case ERR:
        case KEY_RESIZE:
            break;

        case 3:                 /* Ctrl-C */
        case KEY_F(10):
            running = false;
            break;

        case 27:                /* Esc */
            do_stop();
            break;

        case '\n':
        case '\r':
        case KEY_ENTER:
            do_record();
            break;

        case KEY_BACKSPACE:
        case 127:
        case 8:
            if (!rec_active) utf8_pop(name);
            break;

        case 21:                /* Ctrl-U */
            if (!rec_active) name[0] = 0;
            break;

        case KEY_MOUSE: {
            MEVENT me;
            if (getmouse(&me) == OK &&
                (me.bstate & (BUTTON1_CLICKED | BUTTON1_PRESSED))) {
                if (hit(btn_rec, me.y, me.x))       do_record();
                else if (hit(btn_stop, me.y, me.x)) do_stop();
            }
            break;
        }

        default:
            if (!rec_active && ch >= 32 && ch < 256)
                name_append_byte((unsigned char)ch);   /* UTF-8 arrives byte by byte */
            break;
        }

        /* ---- conditions that force a stop ---- */
        if (rec_active) {
            const char *why = NULL;
            if (atomic_load(&server_down))       why = "JACK server shut down - recording stopped";
            else if (atomic_load(&rate_changed)) why = "JACK sample rate changed - recording stopped";
            else if (atomic_load(&limit_hit))    why = "4 GB WAV size limit reached - recording stopped";
            else if (atomic_load(&write_error))  why = "Disk write error - recording stopped";
            if (why) {
                stop_recording();
                shown_secs = current_secs();
                set_status(true, "%s (%s)", why, rec_path);
            }
        }
        if (rec_active) {
            shown_secs = current_secs();
            unsigned long long dr = atomic_load(&dropped_frames);
            if (dr)
                set_status(true, "WARNING: dropped %llu frames (disk too slow)", dr);
        } else if (atomic_load(&server_down) && !down_reported) {
            down_reported = true;
            set_status(true, "JACK server shut down");
        }

        draw();
    }

    /* ---- cleanup ---- */
    endwin();
    stop_recording();       /* also handles SIGTERM/SIGHUP while recording */
    jack_deactivate(client);
    jack_client_close(client);
    jack_ringbuffer_free(ring);

    if (rec_path[0]) {
        char d[32];
        format_duration(d, sizeof d, shown_secs);
        printf("Last recording: %s (%s)\n", rec_path, d);
    }
    return 0;
}
