#include "read_temp.h"
#include "hbridge_peltier.h"

// Global temperature variable
volatile HBridge hb;

// Thread to read temperatures and run peltier continuously
void control_temperature() {
    adc_init();
    adc_set_temp_sensor_enabled(true);

    while (true) {
        hbridge_update_T(&hb, time(NULL), read_peltier_thermistor());
        hbridge_smart_drive(&hb);
        //hbridge_drive(&hb);
        sleep_ms(1000 * hb.t_target / 2);
    }
}

// USB Serial Communication
void usb_serial() {
    char line[32];
    int  pos = 0;
    int ch;
    stdio_init_all();

    while (true) {
        while (!stdio_usb_connected()) {
            sleep_ms(100);  // Wait for USB connection
        }
        ch = getchar_timeout_us(100000);
        if (ch != PICO_ERROR_TIMEOUT) {
            switch (ch) {
                case '9': hb.drive = 0.9; break;
                case '8': hb.drive = 0.8; break;
                case '7': hb.drive = 0.7; break;
                case '6': hb.drive = 0.6; break;
                case '5': hb.drive = 0.5; break;
                case '4': hb.drive = 0.4; break;
                case '3': hb.drive = 0.3; break;
                case '2': hb.drive = 0.2; break;
                case '1': hb.drive = 0.1; break;
                case '0': hb.drive = 0.0; break;
                case '-': hb.drive = -hb.drive; break;
            }
            printf("%f: T=%5.2f C, T_target=%5.2f C, drive=%.2f\n", (float)hb.t_now, hb.T_now, hb.T_target, hb.drive);
            while (getchar_timeout_us(0) >= 0) { }
        }
        sleep_ms(100);
    }
}

// Main function
int main() {
    float T_target=40.0; // C
    float t_target=10.0;  // s
    float gain=0.7;

    hbridge_init(&hb, T_target, t_target, gain);
    multicore_launch_core1(control_temperature); // Launch temperature thread on core 1
    usb_serial();                        // Handle USB communication on core 0
    return 0;
}
