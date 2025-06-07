#include <stdio.h>
#include "hbridge_peltier.h"

// Call once at startup to configure PWM + direction pins
void hbridge_init(HBridge *hb, float T_target, float t_target, float gain) {
    
    // PWM setup
    gpio_set_function(HBRIDGE_PWM_PIN, GPIO_FUNC_PWM);
    hb->hbridge_pwm_slice = pwm_gpio_to_slice_num(HBRIDGE_PWM_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(hb->hbridge_pwm_slice, &cfg, true);

    // Direction pins for motor 1 (Peltier 1)
    gpio_init(HBRIDGE_DIR_PIN1);
    gpio_set_dir(HBRIDGE_DIR_PIN1, GPIO_OUT);
    gpio_init(HBRIDGE_DIR_PIN2);
    gpio_set_dir(HBRIDGE_DIR_PIN2, GPIO_OUT);

    // Direction pins for motor 2 (Peltier 2)
    // gpio_init(HBRIDGE_DIR_PIN3);
    // gpio_set_dir(HBRIDGE_DIR_PIN3, GPIO_OUT);
    // gpio_init(HBRIDGE_DIR_PIN4);
    // gpio_set_dir(HBRIDGE_DIR_PIN4, GPIO_OUT);
    
    // Time and temp targets
    hb->T_prev = hb->T_now = hb->T_target = T_target;
    hb->t_target = t_target;
    hb->t_prev = hb->t_now = time(NULL);
    hb->drive = 0.0;
    hb->gain = gain;
    hb->hysteresis = 1.0f; // ∆T, fine-tune
    hb->active = true;     // starts as engaged, setpoint achieved once
    
    // // === PID gain coefficients ===
    // hb->kp = 1.0f;
    // hb->ki = 0.0f; 
    // hb->kd = 0.0f;
    // hb->pid_integral = 0.0f;
    // hb->pid_prev_error = 0.0f;
}

// Update latest temperature reading and time
void hbridge_update_T(HBridge *hb, time_t t_now, float T_now) {
    hb->t_prev = hb->t_now;
    hb->T_prev = hb->T_now;
    hb->T_now = T_now;
    hb->t_now = t_now;
}

// // === Testing PID control ===
// float hbridge_pid_compute(HBridge *hb, float setpoint, float measured, float dt) {
//     float error = setpoint - measured;
//     hb->pid_integral += error *dt;
//     float derivative = (error - hb->pid_prev_error) / dt;
//     float output = hb->kp*error + hb->ki * hb->pid_integral + hb->kd * derivative;
//     hb->pid_prev_error = error;
//         
//     // clamp output to +/- gain, or hb->gain = 0.2; etc...
//     // NOTE: In hbridge_smart_drive we cap the drive to +/- 0.2, the clamp below
//     // effectively does the same thing, where hb->gain acts as the maximum allowed drive.
//     // The purpose of clamping is to stay below a certain power draw given power budget limitations 
//     // and to maintain stability. 
//     if (output > hb->gain) output = hb->gain;
//     if (output < hb->gain) output = -hb->gain;
//     return output;
// }

// if temp deviates from setpoint by dT, hbridge drives back toward setpoint.
void hbridge_hysteresis_drive(HBridge *hb) {
    
    float error = hb->T_target - hb->T_now;
    
    if (hb->active) {
        if (fabsf(error) <= hb->hysteresis) {
            hb->active = false;                 // goes idle
            hbridge_raw_drive(false, 0);
            hb->drive = 0.0f;
            return;
        }
        hbridge_smart_drive(hb);
    } else {
        // currently on idle - wakes up when we move beyond ∆T
        if (fabsf(error) > hb -> hysteresis) {
            hb->active = true; 
            hbridge_smart_drive(hb);
        } else {
            // stays off
            hb->drive = 0.0f;
            hbridge_raw_drive(false, 0);
        }
    }
}

// === testing hysteresis drive for PID control ===
// void hbridge_hysteresis_drive(HBridge *hb) {
//     float error = hb->T_target - hb->T_now;
//     float dt = (float)(hb->t_now - hb->t_prev); 

//     if (hb->active) {
//         if (fabsf(error) <= hb->hysteresis) {
//             hb->active = false;
//             hbridge_raw_drive(false, 0);
//             hb->drive = 0.0f;
//             return;
//         }
//         // PID control when outside hysteresis window
//         float pid_drive = hbridge_pid_compute(hb, hb->T_target, hb->T_now, dt);
//         hb->drive = pid_drive;
//         hbridge_drive(hb);
//     } else {
//         if (fabsf(error) > hb->hysteresis) {
//             hb->active = true;
//             float pid_drive = hbridge_pid_compute(hb, hb->T_target, hb->T_now, dt);
//             hb->drive = pid_drive;
//             hbridge_drive(hb);
//         } else {
//             hb->drive = 0.0f;
//             hbridge_raw_drive(false, 0);
//         }
//     }
// }


// Drive the hbridge
void hbridge_smart_drive(HBridge *hb) {
    float dT_now, dT_prev;
    float dT_dt;

    // Calculate drive level and direction
    dT_now = hb->T_target - hb->T_now;
    dT_dt = (hb->T_now - hb->T_prev) / (hb->t_now - hb->t_prev);
    if (dT_now > 0) {
        if (dT_now > 0.1) {
            hb->drive =-0.2;
        } else {
            hb->drive = 0.0;
        }
    } else {
        if (dT_now < 0.1) {
            hb->drive = 0.2;
        } else {
            hb->drive = 0.0;
        }
    }
    
    // // test
    // if (0 <= hb->drive < 1e-3) {
    //    hb->drive = 1e-3;
    // } else if (-1e-3 < hb->drive) {
    //    hb->drive = -1e-3;
    // }
    // hb->drive *= hb->gain * (dT_now / hb->t_target) / dT_dt;
    // // end test

    hbridge_drive(hb);
}

void hbridge_drive(HBridge *hb) {
    bool forward;
    uint32_t level;
    forward = hb->drive > 0 ? true : false;
    level = (forward ? hb->drive : -hb->drive) * PWM_WRAP;
    if (level > PWM_WRAP) level = PWM_WRAP;
    hbridge_raw_drive(forward, level);
}

void hbridge_raw_drive(bool forward, uint32_t level) {
    if (level == 0) {
        // printf("Drive: off\n");             // uncomment when sending data to host
        gpio_put(HBRIDGE_DIR_PIN1, false);
        gpio_put(HBRIDGE_DIR_PIN2, false);
    } else {
        level = 0.4 * PWM_WRAP + 0.1 * level;
        // printf("Drive: %b, %d\n", forward, level); // uncomment when sending data to host
        gpio_put(HBRIDGE_DIR_PIN1, forward);
        gpio_put(HBRIDGE_DIR_PIN2, !forward);
    }
    pwm_set_gpio_level(HBRIDGE_PWM_PIN, level);
}


