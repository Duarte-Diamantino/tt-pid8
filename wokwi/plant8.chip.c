// SPDX-License-Identifier: Apache-2.0
//
// plant8.chip.c - first-order plant driven by the 8-bit PID output (Wokwi).
//
// The PID circuit sends the control action as a NUMBER on 8 pins (U7..U0,
// 0..255). The plant reads it every millisecond:
//     tau * dy/dt = gain * U - y - load * DIST
// y (0..255) is output as an analog voltage on Y (0..5 V), which the host
// reads with analogRead and writes back to the PID as the feedback FB.
//
// The display plots the last 2 s: SP (yellow, read from the SP pin),
// y (green) and U (blue).
//
// Controls (click the chip while the simulation runs):
//     tauMs  plant time constant (ms)
//     load   disturbance applied while DIST = 1 (counts)
//     gain   static gain of the plant

#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RGB(r, g, b) (0xFF000000u | ((uint32_t)(b) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(r))
#define C_BG    RGB(16, 20, 24)
#define C_GRID  RGB(48, 56, 64)
#define C_SP    RGB(255, 208, 32)
#define C_Y     RGB(64, 224, 64)
#define C_U     RGB(64, 144, 255)
#define C_LOAD  RGB(255, 80, 80)

#define STEP_US      1000u      // integration / DAC update step
#define PLOT_MS      10u        // one display column every 10 ms
#define HEADER_H     14u
#define VMAX         280.0      // top of the plot (counts)

typedef struct {
    pin_t    pin_u[8], pin_sp, pin_dist, pin_y;
    uint32_t attr_tau, attr_load, attr_gain;
    double   y;                 // plant output (counts)
    uint32_t u;                 // last U read from the pins

    buffer_t  fb;
    uint32_t  w, h;
    uint32_t *pix;
    uint16_t *h_sp, *h_y, *h_u;
    uint32_t  n_hist;
    uint32_t  ms;
} chip_t;

// --------------------------------------------------------------------------
// Display: 3x5 font drawn at 2x
// --------------------------------------------------------------------------
static const char     FONT_CH[]  = "0123456789SPYULOAD-=";
static const uint16_t FONT_BITS[] = {
    // rows top->bottom, 3 bits each (bit2 = left)
    0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9, 0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF,  // 0-9
    0x79CF, 0x7BE4, 0x5BD2, 0x5B6F, 0x4927, 0x7B6F, 0x7BED, 0x6B6E, 0x01C0, 0x0E38   // S P Y U L O A D - =
};

static void px(chip_t *c, int x, int y, uint32_t col) {
    if (x < 0 || y < 0 || x >= (int)c->w || y >= (int)c->h) return;
    c->pix[y * c->w + x] = col;
}

static int draw_char(chip_t *c, int x, int y, char ch, uint32_t col) {
    const char *p = strchr(FONT_CH, ch);
    if (ch != ' ' && p) {
        uint16_t bits = FONT_BITS[p - FONT_CH];
        for (int r = 0; r < 5; r++)
            for (int k = 0; k < 3; k++)
                if (bits & (1u << (14 - (r * 3 + k)))) {
                    px(c, x + 2 * k,     y + 2 * r,     col);
                    px(c, x + 2 * k + 1, y + 2 * r,     col);
                    px(c, x + 2 * k,     y + 2 * r + 1, col);
                    px(c, x + 2 * k + 1, y + 2 * r + 1, col);
                }
    }
    return x + 8;
}

static int draw_text(chip_t *c, int x, int y, const char *s, uint32_t col) {
    while (*s) x = draw_char(c, x, y, *s++, col);
    return x;
}

static int draw_num(chip_t *c, int x, int y, int v, uint32_t col) {
    char tmp[6], buf[8];
    int k = 0, n = 0;
    if (v < 0) { buf[n++] = '-'; v = -v; }
    do { tmp[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 5);
    while (k) buf[n++] = tmp[--k];
    buf[n] = 0;
    return draw_text(c, x, y, buf, col);
}

static int row_of(chip_t *c, double v) {
    if (v < 0) v = 0;
    if (v > VMAX) v = VMAX;
    int plot_h = (int)c->h - (int)HEADER_H;
    return (int)c->h - 1 - (int)(v * (plot_h - 1) / VMAX + 0.5);
}

static void plot_series(chip_t *c, const uint16_t *hst, uint32_t col) {
    int prev = -1;
    for (uint32_t i = 0; i < c->n_hist; i++) {
        int x = (int)(c->w - c->n_hist + i);
        int r = row_of(c, hst[i]);
        int a = prev < 0 ? r : prev;
        int lo = a < r ? a : r, hi = a < r ? r : a;
        for (int y = lo; y <= hi; y++) px(c, x, y, col);
        prev = r;
    }
}

static void redraw(chip_t *c, int sp, int y, int u, int load_on) {
    uint32_t n = c->w * c->h;
    for (uint32_t i = 0; i < n; i++) c->pix[i] = C_BG;
    for (int v = 0; v <= 250; v += 50) {
        int r = row_of(c, v);
        for (uint32_t x = 0; x < c->w; x += (v % 100 ? 4 : 2)) px(c, (int)x, r, C_GRID);
    }
    plot_series(c, c->h_u,  C_U);
    plot_series(c, c->h_sp, C_SP);
    plot_series(c, c->h_y,  C_Y);

    int x = 2;
    x = draw_text(c, x, 2, "SP", C_SP);  x = draw_num(c, x + 4, 2, sp, C_SP);
    x = draw_text(c, x + 8, 2, "Y", C_Y); x = draw_num(c, x + 4, 2, y, C_Y);
    x = draw_text(c, x + 8, 2, "U", C_U); x = draw_num(c, x + 4, 2, u, C_U);
    if (load_on) draw_text(c, (int)c->w - 34, 2, "LOAD", C_LOAD);
    buffer_write(c->fb, 0, c->pix, n * 4);
}

// --------------------------------------------------------------------------
// 1 ms step: read U, integrate, update the DAC, every PLOT_MS add a column
// --------------------------------------------------------------------------
static void on_step(void *user_data) {
    chip_t *c = (chip_t *)user_data;

    uint32_t u = 0;
    for (int k = 0; k < 8; k++) u |= pin_read(c->pin_u[k]) << k;
    c->u = u;

    double tau = attr_read_float(c->attr_tau) * 1e-3;
    if (tau < 1e-3) tau = 1e-3;
    double load   = pin_read(c->pin_dist) ? attr_read_float(c->attr_load) : 0.0;
    double target = attr_read_float(c->attr_gain) * (double)u - load;
    double a = (STEP_US * 1e-6) / tau;          // forward Euler, dt = 1 ms << tau
    if (a > 1.0) a = 1.0;
    c->y += (target - c->y) * a;

    double yo = c->y < 0.0 ? 0.0 : (c->y > 255.0 ? 255.0 : c->y);
    pin_dac_write(c->pin_y, (float)(yo * 5.0 / 255.0));

    if (++c->ms < PLOT_MS) return;
    c->ms = 0;

    int sp = (int)(pin_adc_read(c->pin_sp) * 255.0f / 5.0f + 0.5f);
    int y  = (int)(yo + 0.5);
    if (c->n_hist < c->w) {
        c->n_hist++;
    } else {
        memmove(c->h_sp, c->h_sp + 1, (c->w - 1) * sizeof(uint16_t));
        memmove(c->h_y,  c->h_y  + 1, (c->w - 1) * sizeof(uint16_t));
        memmove(c->h_u,  c->h_u  + 1, (c->w - 1) * sizeof(uint16_t));
    }
    c->h_sp[c->n_hist - 1] = (uint16_t)sp;
    c->h_y [c->n_hist - 1] = (uint16_t)y;
    c->h_u [c->n_hist - 1] = (uint16_t)u;
    redraw(c, sp, y, (int)u, pin_read(c->pin_dist));
}

void chip_init(void) {
    chip_t *c = (chip_t *)calloc(1, sizeof(chip_t));
    static const char *UN[8] = { "U0", "U1", "U2", "U3", "U4", "U5", "U6", "U7" };

    for (int k = 0; k < 8; k++) c->pin_u[k] = pin_init(UN[k], INPUT_PULLDOWN);
    c->pin_dist = pin_init("DIST", INPUT_PULLDOWN);
    c->pin_sp   = pin_init("SP",   ANALOG);
    c->pin_y    = pin_init("Y",    ANALOG);

    c->attr_tau  = attr_init_float("tauMs", 200.0f);
    c->attr_load = attr_init_float("load",   75.0f);
    c->attr_gain = attr_init_float("gain",    1.0f);

    c->fb   = framebuffer_init(&c->w, &c->h);
    c->pix  = (uint32_t *)malloc(c->w * c->h * 4);
    c->h_sp = (uint16_t *)calloc(c->w, sizeof(uint16_t));
    c->h_y  = (uint16_t *)calloc(c->w, sizeof(uint16_t));
    c->h_u  = (uint16_t *)calloc(c->w, sizeof(uint16_t));

    pin_dac_write(c->pin_y, 0.0f);

    const timer_config_t tcfg = { .callback = on_step, .user_data = c };
    timer_t t = timer_init(&tcfg);
    timer_start(t, STEP_US, true);

    redraw(c, 0, 0, 0, 0);
}
