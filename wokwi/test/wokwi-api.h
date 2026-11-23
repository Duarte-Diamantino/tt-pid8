// Minimal stand-in for Wokwi's wokwi-api.h, from the signatures in docs.wokwi.com/chips-api.
// Used only to compile-check plant.chip.c and run it natively.
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef int32_t  pin_t;
typedef uint32_t timer_t_;
#define timer_t timer_t_
typedef uint32_t buffer_t;
enum { LOW = 0, HIGH = 1 };
enum { INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2, INPUT_PULLDOWN = 3, ANALOG = 4, OUTPUT_LOW = 16, OUTPUT_HIGH = 17 };
enum { BOTH = 0, RISING = 1, FALLING = 2 };
typedef struct { void *user_data; uint32_t edge; void (*pin_change)(void *user_data, pin_t pin, uint32_t value); } pin_watch_config_t;
typedef struct { void *user_data; void (*callback)(void *user_data); } timer_config_t;
pin_t    pin_init(const char *name, uint32_t mode);
void     pin_mode(pin_t pin, uint32_t mode);
void     pin_write(pin_t pin, uint32_t value);
uint32_t pin_read(pin_t pin);
bool     pin_watch(pin_t pin, const pin_watch_config_t *config);
float    pin_adc_read(pin_t pin);
void     pin_dac_write(pin_t pin, float voltage);
uint64_t get_sim_nanos(void);
timer_t  timer_init(const timer_config_t *config);
void     timer_start(timer_t timer_id, uint32_t micros, bool repeat);
uint32_t attr_init(const char *name, uint32_t default_value);
uint32_t attr_init_float(const char *name, float default_value);
uint32_t attr_read(uint32_t attr_id);
float    attr_read_float(uint32_t attr_id);
buffer_t framebuffer_init(uint32_t *pixel_width, uint32_t *pixel_height);
void     buffer_write(buffer_t buffer, uint32_t offset, void *data, uint32_t data_len);
