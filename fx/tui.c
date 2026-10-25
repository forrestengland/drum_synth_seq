/*
 * tui.c - ncurses front end (drop-in replacement for ui.c; same ui_run()).
 *
 * Keys:
 *   Up/Down, j/k   select parameter (wraps across effects)
 *   Left/Right,h/l adjust (2% of travel)     PgUp/PgDn  coarse (10%)
 *   , .            fine (0.2%)               r          reset to default
 *   b, Space, Enter toggle bypass of selected effect
 *   Tab            jump to next effect       q          quit
 * Mouse (if the terminal reports it): click/drag a bar to set it, wheel over
 * a bar to adjust, right-click a bar to reset, click the bypass button.
 *
 * JACK and ncurses both want stderr, so it's redirected to fxrack.log for
 * the lifetime of the UI.
 */
#include "ui.h"

#include <curses.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MAX_HITS 128
#define MAX_BAR  100

enum { CP_ACCENT = 1, CP_GOOD, CP_BAD, CP_TITLE, CP_DIM };

typedef struct { int y, x0, x1, fx, p; } Hit;   /* p == -1: bypass button */

static Hit hits[MAX_HITS];
static int nhits;
static volatile sig_atomic_t *g_quit;

static void on_signal(int sig)
{
    (void)sig;
    if (g_quit) *g_quit = 1;
}

static void add_hit(int y, int x0, int x1, int fx, int p)
{
    if (nhits < MAX_HITS)
        hits[nhits++] = (Hit){ y, x0, x1, fx, p };
}

static void nudge(Param *p, float d)
{
    param_set(p, param_from_norm(p, param_to_norm(p, param_get(p)) + d));
}

static void toggle_bypass(Effect *fx)
{
    int cur = atomic_load_explicit(&fx->bypass, memory_order_relaxed);
    atomic_store_explicit(&fx->bypass, !cur, memory_order_relaxed);
}

/* ---------- drawing ---------- */

static void draw_effect(Effect *fx, int e, int *y, int width, int sel_fx, int sel_p)
{
    int bypassed = atomic_load_explicit(&fx->bypass, memory_order_relaxed);

    /* header: name ........ [ ACTIVE ] */
    attron(COLOR_PAIR(CP_TITLE) | A_BOLD);
    mvhline(*y, 1, ACS_HLINE, width - 2);
    mvprintw(*y, 2, " %s ", fx->name);
    attroff(COLOR_PAIR(CP_TITLE) | A_BOLD);

    const char *btn = bypassed ? "[ BYPASSED ]" : "[  ACTIVE  ]";
    int bx = width - (int)strlen(btn) - 2;
    attron(COLOR_PAIR(bypassed ? CP_BAD : CP_GOOD) | A_BOLD | (bypassed ? A_REVERSE : 0));
    mvaddstr(*y, bx, btn);
    attroff(COLOR_PAIR(bypassed ? CP_BAD : CP_GOOD) | A_BOLD | A_REVERSE);
    add_hit(*y, bx, bx + (int)strlen(btn), e, -1);
    (*y)++;

    int bw = width - 36;                /* marker + label + brackets + value */
    if (bw < 8) bw = 8;
    if (bw > MAX_BAR) bw = MAX_BAR;

    for (int p = 0; p < fx->nparams; p++, (*y)++) {
        Param *pr = &fx->params[p];
        float v = param_get(pr);
        float norm = param_to_norm(pr, v);
        int fill = (int)(norm * bw + 0.5f);
        int selected = (e == sel_fx && p == sel_p);

        char bar[MAX_BAR + 1], val[32];
        for (int i = 0; i < bw; i++) bar[i] = i < fill ? '#' : '-';
        bar[bw] = '\0';
        snprintf(val, sizeof val, pr->fmt, v);

        if (selected) attron(A_BOLD);
        mvprintw(*y, 2, "%c %-9s", selected ? '>' : ' ', pr->name);
        if (selected) attroff(A_BOLD);

        int bar_x = 14;
        mvaddch(*y, bar_x, '[');
        attron(COLOR_PAIR(bypassed ? CP_DIM : CP_ACCENT) | (selected ? A_BOLD : 0));
        mvaddstr(*y, bar_x + 1, bar);
        attroff(COLOR_PAIR(bypassed ? CP_DIM : CP_ACCENT) | A_BOLD);
        mvaddch(*y, bar_x + 1 + bw, ']');
        mvprintw(*y, bar_x + 3 + bw, "%s", val);

        add_hit(*y, bar_x + 1, bar_x + bw, e, p);
    }
    (*y)++;                              /* blank line between effects */
}

static void draw(Effect **rack, int nfx, int sel_fx, int sel_p)
{
    erase();
    nhits = 0;
    int width = COLS < 80 ? COLS : 80;
    int y = 0;

    attron(A_BOLD);
    mvaddstr(y++, 1, "fxrack - JACK 2 in / 2 out");
    attroff(A_BOLD);
    y++;

    for (int e = 0; e < nfx; e++)
        draw_effect(rack[e], e, &y, width, sel_fx, sel_p);

    attron(A_DIM);
    mvaddnstr(LINES - 1, 0,
              "arrows select/adjust  PgUp/PgDn coarse  , . fine  r reset  "
              "b bypass  Tab next  q quit",
              COLS - 1);
    attroff(A_DIM);
    refresh();
}

/* ---------- input ---------- */

static void set_from_x(Effect **rack, const Hit *h, int x)
{
    Param *p = &rack[h->fx]->params[h->p];
    int span = h->x1 - h->x0;
    float n = span > 0 ? (float)(x - h->x0) / (float)span : 0.0f;
    param_set(p, param_from_norm(p, n));
}

static const Hit *hit_at(int y, int x)
{
    for (int i = 0; i < nhits; i++)
        if (hits[i].y == y && x >= hits[i].x0 - 1 && x <= hits[i].x1 + 1)
            return &hits[i];
    return NULL;
}

static void handle_mouse(Effect **rack, int *sel_fx, int *sel_p, int *drag)
{
    MEVENT m;
    if (getmouse(&m) != OK) return;

    if (m.bstate & BUTTON1_RELEASED) { *drag = -1; return; }

    if (m.bstate & (BUTTON1_PRESSED | BUTTON1_CLICKED)) {
        const Hit *h = hit_at(m.y, m.x);
        if (!h) return;
        if (h->p < 0) { toggle_bypass(rack[h->fx]); return; }
        *sel_fx = h->fx; *sel_p = h->p;
        set_from_x(rack, h, m.x);
        *drag = (int)(h - hits);
        return;
    }

    if ((m.bstate & REPORT_MOUSE_POSITION) && *drag >= 0 && *drag < nhits) {
        set_from_x(rack, &hits[*drag], m.x);     /* drag may leave the row */
        return;
    }

    if (m.bstate & BUTTON3_PRESSED) {
        const Hit *h = hit_at(m.y, m.x);
        if (h && h->p >= 0) {
            Param *p = &rack[h->fx]->params[h->p];
            param_set(p, p->def);
        }
        return;
    }

    float wheel = 0.0f;
    if (m.bstate & BUTTON4_PRESSED) wheel = 0.02f;
#ifdef BUTTON5_PRESSED
    if (m.bstate & BUTTON5_PRESSED) wheel = -0.02f;
#endif
    if (wheel != 0.0f) {
        const Hit *h = hit_at(m.y, m.x);
        if (h && h->p >= 0) nudge(&rack[h->fx]->params[h->p], wheel);
    }
}

static void move_sel(Effect **rack, int nfx, int *sel_fx, int *sel_p, int d)
{
    *sel_p += d;
    if (*sel_p < 0) {
        *sel_fx = (*sel_fx + nfx - 1) % nfx;
        *sel_p = rack[*sel_fx]->nparams - 1;
    } else if (*sel_p >= rack[*sel_fx]->nparams) {
        *sel_fx = (*sel_fx + 1) % nfx;
        *sel_p = 0;
    }
}

/* ---------- main loop ---------- */

int ui_run(Effect **rack, int nfx, volatile sig_atomic_t *quit)
{
    if (nfx < 1) return 0;
    g_quit = quit;

    /* keep JACK's chatter off the screen */
    fflush(stderr);
    int saved_err = dup(2);
    int logfd = open("fxrack.log", O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (logfd >= 0) { dup2(logfd, 2); close(logfd); }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;          /* no SA_RESTART: getch() wakes up */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(50);                        /* redraw ~20x/s even when idle */
    mouseinterval(0);                   /* report press/release immediately */
    mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);

    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(CP_ACCENT, COLOR_YELLOW, -1);
        init_pair(CP_GOOD,   COLOR_GREEN,  -1);
        init_pair(CP_BAD,    COLOR_RED,    -1);
        init_pair(CP_TITLE,  COLOR_CYAN,   -1);
        init_pair(CP_DIM,    COLOR_WHITE,  -1);
    }

    int sel_fx = 0, sel_p = 0, drag = -1, running = 1;

    while (running && !*quit) {
        draw(rack, nfx, sel_fx, sel_p);

        int ch = getch();
        Effect *fx = rack[sel_fx];
        Param *p = &fx->params[sel_p];

        switch (ch) {
        case ERR: case KEY_RESIZE: break;
        case 'q': case 'Q':        running = 0; break;
        case KEY_UP:   case 'k':   move_sel(rack, nfx, &sel_fx, &sel_p, -1); break;
        case KEY_DOWN: case 'j':   move_sel(rack, nfx, &sel_fx, &sel_p, +1); break;
        case '\t':                 sel_fx = (sel_fx + 1) % nfx; sel_p = 0; break;
        case KEY_LEFT:  case 'h':  nudge(p, -0.02f); break;
        case KEY_RIGHT: case 'l':  nudge(p, +0.02f); break;
        case KEY_NPAGE:            nudge(p, -0.10f); break;
        case KEY_PPAGE:            nudge(p, +0.10f); break;
        case ',':                  nudge(p, -0.002f); break;
        case '.':                  nudge(p, +0.002f); break;
        case 'r': case 'R':        param_set(p, p->def); break;
        case 'b': case 'B': case ' ': case '\n': case KEY_ENTER:
                                   toggle_bypass(fx); break;
        case KEY_MOUSE:            handle_mouse(rack, &sel_fx, &sel_p, &drag); break;
        }
    }

    endwin();
    fflush(stderr);
    if (saved_err >= 0) { dup2(saved_err, 2); close(saved_err); }
    return 0;
}
