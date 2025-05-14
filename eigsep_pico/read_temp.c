#include "read_temp.h"

// Function to read the internal pico temperature sensor
float read_pico_temperature() {
    adc_select_input(ADC_INTERNAL_PICO);
    uint16_t raw = adc_read();
    float temp_c = 27.0f - (raw * adc_v_per_cnt - 0.706f) / 0.001721f;
    return temp_c;
}

// Function to read the external thermistor temperature sensor
float read_peltier_thermistor() {
    adc_select_input(ADC_THERMISTOR);
    uint16_t raw = adc_read();
    float vout = raw * adc_v_per_cnt;
    float logr_therm = logf(NTC_R * (vout / (ADC_V - vout)));  // log ohm
    float inv_T = STEIN_A + STEIN_B * logr_therm + STEIN_C * pow(logr_therm, 3);
    return 1 / inv_T - ZEROC_IN_K;
}
