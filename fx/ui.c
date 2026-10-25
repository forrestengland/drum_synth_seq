/*
 * ui.c - SDL2 front end. One panel per effect, generated from its params:
 *   - knobs: drag up/down, mouse wheel, shift = fine, double-click = reset
 *   - bypass button in the panel header
 * Needs SDL2_ttf and a TTF font (DejaVu Sans by default; override with the
 * FXRACK_FONT environment variable).
 */
#include "ui.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_F      3.14159265f
#define PANEL_H   210
#define HEADER_H  40
#define KNOB_R    34
#define COL_W     110
#define MIN_W     480

static const SDL_Color C_BG     = { 18,  19,  22, 255 };
static const SDL_Color C_PANEL  = { 38,  40,  46, 255 };
static const SDL_Color C_HEAD   = { 26,  27,  31, 255 };
static const SDL_Color C_TEXT   = { 225, 225, 230, 255 };
static const SDL_Color C_DIM    = { 125, 127, 135, 255 };
static const SDL_Color C_ACCENT = { 255, 170,  40, 255 };
static const SDL_Color C_TRACK  = { 22,  23,  26, 255 };
static const SDL_Color C_KNOB   = { 64,  66,  74, 255 };
static const SDL_Color C_BTN    = { 52,  54,  61, 255 };
static const SDL_Color C_BTN_ON = { 130,  40,  34, 255 };
static const SDL_Color C_GREEN  = { 80, 215, 110, 255 };
static const SDL_Color C_LEDOFF = { 40,  70,  48, 255 };

typedef struct {
    Effect **rack;
    int nfx;
    int w;
    int drag_fx, drag_p;    /* knob being dragged, or -1 */
} Ui;

/* ---------- drawing helpers ---------- */

static void set_color(SDL_Renderer *r, SDL_Color c)
{
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

static void fill_circle(SDL_Renderer *r, int cx, int cy, int rad)
{
    for (int dy = -rad; dy <= rad; dy++) {
        int dx = (int)sqrtf((float)(rad * rad - dy * dy));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

static void draw_arc(SDL_Renderer *r, int cx, int cy,
                     float r_in, float r_out, float a0, float a1)
{
    for (float a = a0; a <= a1; a += 0.01f) {
        float c = cosf(a), s = sinf(a);
        SDL_RenderDrawLine(r,
            cx + (int)lroundf(c * r_in),  cy + (int)lroundf(s * r_in),
            cx + (int)lroundf(c * r_out), cy + (int)lroundf(s * r_out));
    }
}

/* align: -1 left, 0 centre, 1 right */
static void draw_text(SDL_Renderer *r, TTF_Font *f, const char *s,
                      int x, int y, SDL_Color c, int align)
{
    if (!f || !s || !*s) return;
    SDL_Surface *surf = TTF_RenderUTF8_Blended(f, s, c);
    if (!surf) return;
    SDL_Texture *t = SDL_CreateTextureFromSurface(r, surf);
    if (t) {
        SDL_Rect dst = { x, y, surf->w, surf->h };
        if (align == 0) dst.x -= surf->w / 2;
        else if (align > 0) dst.x -= surf->w;
        SDL_RenderCopy(r, t, NULL, &dst);
        SDL_DestroyTexture(t);
    }
    SDL_FreeSurface(surf);
}

/* ---------- layout ---------- */

static void knob_center(const Ui *ui, int e, int p, int *cx, int *cy)
{
    int n = ui->rack[e]->nparams;
    int x0 = (ui->w - n * COL_W) / 2;
    *cx = x0 + p * COL_W + COL_W / 2;
    *cy = e * PANEL_H + HEADER_H + 24 + KNOB_R;
}

static SDL_Rect bypass_rect(const Ui *ui, int e)
{
    SDL_Rect r = { ui->w - 116, e * PANEL_H + 8, 104, 24 };
    return r;
}

static int find_knob(const Ui *ui, int x, int y, int *fe, int *fp)
{
    for (int e = 0; e < ui->nfx; e++) {
        for (int p = 0; p < ui->rack[e]->nparams; p++) {
            int cx, cy;
            knob_center(ui, e, p, &cx, &cy);
            int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= (KNOB_R + 8) * (KNOB_R + 8)) {
                *fe = e; *fp = p;
                return 1;
            }
        }
    }
    return 0;
}

/* ---------- drawing ---------- */

static void draw_knob(SDL_Renderer *r, TTF_Font *small, Param *p,
                      int cx, int cy, int bypassed)
{
    const float A0 = 0.75f * PI_F;      /* 135 deg: bottom-left */
    const float SWEEP = 1.5f * PI_F;    /* 270 deg of travel    */
    float v = param_get(p);
    float norm = param_to_norm(p, v);
    float a = A0 + SWEEP * norm;

    set_color(r, C_TRACK);
    draw_arc(r, cx, cy, KNOB_R + 3, KNOB_R + 8, A0, A0 + SWEEP);
    set_color(r, bypassed ? C_DIM : C_ACCENT);
    draw_arc(r, cx, cy, KNOB_R + 3, KNOB_R + 8, A0, a);

    set_color(r, C_KNOB);
    fill_circle(r, cx, cy, KNOB_R);

    /* pointer, 3px wide */
    float c = cosf(a), s = sinf(a);
    set_color(r, C_TEXT);
    for (int k = -1; k <= 1; k++) {
        float ox = -s * k, oy = c * k;
        SDL_RenderDrawLine(r,
            cx + (int)lroundf(c * 8 + ox),            cy + (int)lroundf(s * 8 + oy),
            cx + (int)lroundf(c * (KNOB_R - 4) + ox), cy + (int)lroundf(s * (KNOB_R - 4) + oy));
    }

    char buf[32];
    snprintf(buf, sizeof buf, p->fmt, v);
    draw_text(r, small, p->name, cx, cy + KNOB_R + 16, C_DIM, 0);
    draw_text(r, small, buf,     cx, cy + KNOB_R + 36, bypassed ? C_DIM : C_ACCENT, 0);
}

static void draw_panel(SDL_Renderer *r, const Ui *ui, int e,
                       TTF_Font *big, TTF_Font *small)
{
    Effect *fx = ui->rack[e];
    int y = e * PANEL_H;
    int bypassed = atomic_load_explicit(&fx->bypass, memory_order_relaxed);

    SDL_Rect body = { 0, y, ui->w, PANEL_H };
    SDL_Rect head = { 0, y, ui->w, HEADER_H };
    set_color(r, C_PANEL); SDL_RenderFillRect(r, &body);
    set_color(r, C_HEAD);  SDL_RenderFillRect(r, &head);
    set_color(r, C_BG);
    SDL_RenderDrawLine(r, 0, y + PANEL_H - 1, ui->w, y + PANEL_H - 1);

    draw_text(r, big, fx->name, 16, y + 9, bypassed ? C_DIM : C_TEXT, -1);

    /* bypass button + LED (green = effect active) */
    SDL_Rect b = bypass_rect(ui, e);
    set_color(r, bypassed ? C_BTN_ON : C_BTN);
    SDL_RenderFillRect(r, &b);
    set_color(r, C_TRACK);
    SDL_RenderDrawRect(r, &b);
    set_color(r, bypassed ? C_LEDOFF : C_GREEN);
    fill_circle(r, b.x + 14, b.y + b.h / 2, 5);
    draw_text(r, small, "BYPASS", b.x + 28, b.y + 4, C_TEXT, -1);

    for (int p = 0; p < fx->nparams; p++) {
        int cx, cy;
        knob_center(ui, e, p, &cx, &cy);
        draw_knob(r, small, &fx->params[p], cx, cy, bypassed);
    }

    /* rack screws */
    set_color(r, C_TRACK);
    fill_circle(r, 12, y + PANEL_H - 14, 4);
    fill_circle(r, ui->w - 12, y + PANEL_H - 14, 4);
}

/* ---------- input ---------- */

static void nudge(Param *p, float delta_norm)
{
    float n = param_to_norm(p, param_get(p));
    param_set(p, param_from_norm(p, n + delta_norm));
}

static void handle_event(Ui *ui, const SDL_Event *ev, int *running)
{
    int fine = (SDL_GetModState() & KMOD_SHIFT) != 0;

    switch (ev->type) {
    case SDL_QUIT:
        *running = 0;
        break;

    case SDL_MOUSEBUTTONDOWN: {
        if (ev->button.button != SDL_BUTTON_LEFT) break;
        SDL_Point pt = { ev->button.x, ev->button.y };

        for (int e = 0; e < ui->nfx; e++) {
            SDL_Rect b = bypass_rect(ui, e);
            if (SDL_PointInRect(&pt, &b)) {
                Effect *fx = ui->rack[e];
                int cur = atomic_load_explicit(&fx->bypass, memory_order_relaxed);
                atomic_store_explicit(&fx->bypass, !cur, memory_order_relaxed);
                return;
            }
        }

        int fe, fp;
        if (find_knob(ui, pt.x, pt.y, &fe, &fp)) {
            Param *p = &ui->rack[fe]->params[fp];
            if (ev->button.clicks >= 2) {
                param_set(p, p->def);               /* double-click: reset */
            } else {
                ui->drag_fx = fe;
                ui->drag_p = fp;
                SDL_CaptureMouse(SDL_TRUE);         /* keep drag outside window */
            }
        }
        break;
    }

    case SDL_MOUSEBUTTONUP:
        if (ev->button.button == SDL_BUTTON_LEFT && ui->drag_fx >= 0) {
            ui->drag_fx = -1;
            SDL_CaptureMouse(SDL_FALSE);
        }
        break;

    case SDL_MOUSEMOTION:
        if (ui->drag_fx >= 0) {
            Param *p = &ui->rack[ui->drag_fx]->params[ui->drag_p];
            nudge(p, -ev->motion.yrel * (fine ? 0.0005f : 0.005f));
        }
        break;

    case SDL_MOUSEWHEEL: {
        int mx, my, fe, fp;
        SDL_GetMouseState(&mx, &my);
        if (find_knob(ui, mx, my, &fe, &fp))
            nudge(&ui->rack[fe]->params[fp], ev->wheel.y * (fine ? 0.002f : 0.02f));
        break;
    }
    }
}

/* ---------- setup / main loop ---------- */

static TTF_Font *open_font(int pt)
{
    const char *paths[] = {
        getenv("FXRACK_FONT"),
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    };
    for (size_t i = 0; i < sizeof paths / sizeof paths[0]; i++) {
        if (!paths[i]) continue;
        TTF_Font *f = TTF_OpenFont(paths[i], pt);
        if (f) return f;
    }
    return NULL;
}

int ui_run(Effect **rack, int nfx, volatile sig_atomic_t *quit)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    if (TTF_Init() != 0) {
        fprintf(stderr, "TTF_Init: %s\n", TTF_GetError());
        SDL_Quit();
        return 1;
    }

    Ui ui = { .rack = rack, .nfx = nfx, .w = MIN_W, .drag_fx = -1, .drag_p = -1 };
    for (int e = 0; e < nfx; e++) {
        int need = rack[e]->nparams * COL_W + 40;
        if (need > ui.w) ui.w = need;
    }
    int h = (nfx > 0 ? nfx : 1) * PANEL_H;

    SDL_Window *win = SDL_CreateWindow("fxrack", SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED, ui.w, h, 0);
    SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, 0) : NULL;
    TTF_Font *small = open_font(12);
    TTF_Font *big = open_font(18);
    if (!win || !ren) {
        fprintf(stderr, "SDL window/renderer: %s\n", SDL_GetError());
        goto out;
    }
    if (!small || !big)
        fprintf(stderr, "warning: no TTF font found, labels disabled "
                        "(install fonts-dejavu-core or set FXRACK_FONT)\n");

    int running = 1;
    while (running && !*quit) {
        SDL_Event ev;
        if (SDL_WaitEventTimeout(&ev, 50)) {
            do { handle_event(&ui, &ev, &running); } while (SDL_PollEvent(&ev));
        }

        set_color(ren, C_BG);
        SDL_RenderClear(ren);
        for (int e = 0; e < nfx; e++)
            draw_panel(ren, &ui, e, big, small);
        SDL_RenderPresent(ren);
    }

out:
    if (small) TTF_CloseFont(small);
    if (big) TTF_CloseFont(big);
    if (ren) SDL_DestroyRenderer(ren);
    if (win) SDL_DestroyWindow(win);
    TTF_Quit();
    SDL_Quit();
    return 0;
}
