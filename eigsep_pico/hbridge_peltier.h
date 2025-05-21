#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h> // for fabsf in hbridge_hysteresis_drive

// === PELTIER-1 CONFIGURATION ===
#define HBRIDGE_PWM_PIN    16    // PWM input on H‑bridge
#define HBRIDGE_DIR_PIN1   18    // H‑bridge DIR pin A
#define HBRIDGE_DIR_PIN2   19    // H‑bridge DIR pin B

// === PELTIER-2 CONFIGURATION ===
// #define HBRIDGE_PWM_ENA2     // PWM input for motor 2
// #define HBRIDGE_DIR_PIN3     // H-bridge DIR pin A, i.e. IN3
// #define HBRIDGE_DIR_PIN4     // H-bridge DIR pin B, i.e. IN4

// === PWM ===
# define PWM_WRAP       1000 // PWM steps; 1000 gives 0.1% resolution

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
    float hysteresis; // ∆T, allow the temp. to deviate from the setpoint by x˚C - the intent is to achieve the setpoint -> TEC "turns off" until ∆T deviation
    bool active;  // when true, PID is engaged
    bool enabled; // if we want to enable/disable H-bridge at runtime -- used in runtime_cmd.c line 28/34
    
} HBridge;

void hbridge_init(HBridge *hb, float T_target, float t_target, float gain);
void hbridge_update_T(HBridge *hb, time_t t_now, float T_now);
void hbridge_hysteresis_drive(HBridge *hb);     
void hbridge_smart_drive(HBridge *hb);
void hbridge_drive(HBridge *hb);
void hbridge_raw_drive(bool forward, uint32_t level);
