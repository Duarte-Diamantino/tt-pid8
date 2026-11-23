// SPDX-License-Identifier: Apache-2.0
//
// Native harness for plant8.chip.c (uses wokwi-api.h, a minimal stand-in for the Wokwi chip API).
// Emulates the Wokwi runtime (time, one repeating timer, 8 U pins, DIST,
// attributes, framebuffer) and checks, through the DAC voltage on Y:
//   * the step response to U = 153 with tau = 200 ms;
//   * y settles at 153;
//   * DIST = 1 with load 75: y -> 78.
// Run: gcc -std=c11 -Wall -I wokwi/test -o plant8_host wokwi/test/plant8_host.c wokwi/plant8.chip.c -lm
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void chip_init(void);

static uint64_t now_ns;
static void (*tmr_cb)(void *);
static void *tmr_ud;
static uint64_t tmr_period, tmr_next;
static uint32_t u_val, lvl_dist;
static float dac_v, sp_v = 2.5f;
static uint32_t W, H, *fbuf;
static float attrs[3];

// pins: U0..U7 = 0..7, DIST = 8, SP = 9, Y = 10
pin_t pin_init(const char *n, uint32_t m) {
    (void)m;
    if (n[0] == 'U') return n[1] - '0';
    return !strcmp(n, "DIST") ? 8 : !strcmp(n, "SP") ? 9 : 10;
}
void pin_mode(pin_t p, uint32_t m) { (void)p; (void)m; }
void pin_write(pin_t p, uint32_t v) { (void)p; (void)v; }
uint32_t pin_read(pin_t p) { return p < 8 ? (u_val >> p) & 1 : p == 8 ? lvl_dist : 0; }
bool pin_watch(pin_t p, const pin_watch_config_t *c) { (void)p; (void)c; return true; }
float pin_adc_read(pin_t p) { (void)p; return sp_v; }
void pin_dac_write(pin_t p, float v) { (void)p; dac_v = v; }
uint64_t get_sim_nanos(void) { return now_ns; }
timer_t timer_init(const timer_config_t *c) { tmr_cb = c->callback; tmr_ud = c->user_data; return 1; }
void timer_start(timer_t t, uint32_t us, bool r) { (void)t; (void)r; tmr_period = us * 1000ull; tmr_next = now_ns + tmr_period; }
uint32_t attr_init(const char *n, uint32_t d) { (void)n; (void)d; return 0; }
static int attr_id(const char *n) { return !strcmp(n, "tauMs") ? 0 : !strcmp(n, "load") ? 1 : 2; }
uint32_t attr_init_float(const char *n, float d) { attrs[attr_id(n)] = d; return attr_id(n); }
uint32_t attr_read(uint32_t a) { (void)a; return 0; }
float attr_read_float(uint32_t a) { return attrs[a]; }
buffer_t framebuffer_init(uint32_t *w, uint32_t *h) { W = *w = 200; H = *h = 120; fbuf = calloc(W * H, 4); return 1; }
void buffer_write(buffer_t b, uint32_t off, void *d, uint32_t len) { (void)b; memcpy((char *)fbuf + off, d, len); }

static void run_until(uint64_t t_end) {
    while (tmr_next <= t_end) { now_ns = tmr_next; tmr_cb(tmr_ud); tmr_next += tmr_period; }
    now_ns = t_end;
}
static double y_counts(void) { return dac_v * 255.0 / 5.0; }

int main(void) {
    int errors = 0;
    chip_init();
    u_val = 153;
    run_until(200000000ull);                                  // one tau
    double y1 = y_counts(), e1 = 153 * (1 - exp(-1.0));
    printf("t=0.2 s y=%.1f (expect ~%.1f = 153*(1-e^-1))\n", y1, e1);
    if (fabs(y1 - e1) > 2) errors++;
    run_until(2000000000ull);
    double y2 = y_counts();
    printf("t=2 s   y=%.1f (expect 153)\n", y2);
    if (fabs(y2 - 153) > 0.5) errors++;
    lvl_dist = 1;
    run_until(4000000000ull);
    double y3 = y_counts();
    printf("load    y=%.1f (expect 78 = 153 - 75)\n", y3);
    if (fabs(y3 - 78) > 0.5) errors++;
    printf(errors ? "FAIL plant8\n" : "PASS plant8\n");
    return errors != 0;
}
