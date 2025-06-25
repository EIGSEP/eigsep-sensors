#include "read_temp.h"

// === ds18b20 === 
#include "onewire_library.h"
#include "ds18b20.h"
#include "ow_rom.h"

extern OW ow;


// Function to read the internal pico temperature sensor
float read_pico_temperature() {
    adc_select_input(ADC_INTERNAL_PICO);
    uint16_t raw = adc_read();
    float temp_c = 27.0f - (raw * adc_v_per_cnt - 0.706f) / 0.001721f;
    return temp_c;
}

// // Function to read the external thermistor temperature sensor -- old thermistors
// float read_peltier_thermistor() {
//     adc_select_input(ADC_THERMISTOR); 
//     uint16_t raw = adc_read();
//     float vout = raw * adc_v_per_cnt;
//     float logr_therm = logf(NTC_R * (vout / (ADC_V - vout)));  // log ohm
//     float inv_T = STEIN_A + STEIN_B * logr_therm + STEIN_C * pow(logr_therm, 3);
//     return (1 / inv_T) - ZEROC_IN_K;
// }


float read_ds18b20_celsius(void) {
    ow_reset(&ow);
    ow_send(&ow, OW_SKIP_ROM);          // broadcast (assumes one sensor)
    ow_send(&ow, DS18B20_CONVERT_T);    // 0x44

    ow_reset(&ow);
    ow_send(&ow, OW_SKIP_ROM);
    ow_send(&ow, DS18B20_READ_SCRATCHPAD);

    uint8_t scratch[9];
    for (int i = 0; i < 9; ++i)
        scratch[i] = ow_read(&ow);
    // Parse temperature
    int16_t raw = (scratch[1] << 8) | scratch[0];
    float temp_celsius = (float)raw / 16.0f;
    return temp_celsius;
}

// float read_ds18b20_celsius(void) {
//     printf("[DS18B20] Starting temperature read...\n");

//     if (!ow_reset(&ow)) {
//         printf("[DS18B20] ERROR: Sensor not responding (reset failed)\n");
//         return -1000.0f; // Error value
//     }
//     ow_send(&ow, OW_SKIP_ROM);
//     ow_send(&ow, DS18B20_CONVERT_T);

//     sleep_ms(750);

//     if (!ow_reset(&ow)) {
//         printf("[DS18B20] ERROR: Sensor not responding after conversion (reset failed)\n");
//         return -1000.0f;
//     }
//     ow_send(&ow, OW_SKIP_ROM);
//     ow_send(&ow, DS18B20_READ_SCRATCHPAD);

//     uint8_t scratch[9];
//     for (int i = 0; i < 9; ++i) {
//         scratch[i] = ow_read(&ow);
//         printf("[DS18B20] Scratch[%d] = 0x%02X\n", i, scratch[i]);
//     }

//     // Optional: check CRC if available

//     int16_t raw = (scratch[1] << 8) | scratch[0];
//     float temp_celsius = (float)raw / 16.0f;
//     printf("[DS18B20] Returning %.2f C (raw=0x%04X)\n", temp_celsius, raw);
//     return temp_celsius;
// }
