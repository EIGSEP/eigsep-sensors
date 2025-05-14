#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// === PELTIER CONFIGURATION ===
#define HBRIDGE_PWM_PIN    16    // PWM input on H‑bridge
#define HBRIDGE_DIR_PIN1   18    // H‑bridge DIR pin A
#define HBRIDGE_DIR_PIN2   19    // H‑bridge DIR pin B
#define PWM_WRAP         1000   // PWM steps; 1000 gives 0.1% resolution



typedef struct {
    // configuration
    uint hbridge_pwm_slice;

    // runtime state
    float T_now;
    time_t t_now;
    float T_prev;
    time_t t_prev;
    float T_target;
    float t_target;
    float drive;
    float gain;
} HBridge;

void hbridge_init(HBridge *hb, float T_target, float t_target, float gain);
void hbridge_update_T(HBridge *hb, time_t t_now, float T_now);
void hbridge_smart_drive(HBridge *hb);
void hbridge_drive(HBridge *hb);
void hbridge_raw_drive(bool forward, uint32_t level);
