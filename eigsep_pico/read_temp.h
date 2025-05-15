#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include <string.h> // for logger, usb_serial_request_reply in main.c

// ADC targets
#define ADC_INTERNAL_PICO 4
#define ADC_THERMISTOR 0  // Pin 26
#define ADC_V 3.3f
#define ADC_BITS 12
#define ZEROC_IN_K 273.15

// === THERMISTOR CALIBRATION ===
#define NTC_R 10000.0f
#define NTC_BETA 3950.0f
#define STEIN_A 1.0463e-03
#define STEIN_B 2.4916e-04
#define STEIN_C 1.6436e-08

// === Commands ===
#define ETX 0x03 // ctrl + c byte

static const float adc_v_per_cnt = ADC_V / (1 << ADC_BITS);

float read_pico_temperature();
float read_peltier_thermistor();
