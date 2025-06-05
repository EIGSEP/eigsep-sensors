#include "read_temp.h"
#include "hbridge_peltier.h"
#include "runtime_cmd.h"

// Global temperature variable
volatile HBridge hb;
// HBridge hb;

// callback for repeating timer 
bool control_temperature_callback(struct repeating_timer *t) {
    hbridge_update_T(&hb, time(NULL), read_peltier_thermistor());
    hbridge_hysteresis_drive(&hb);
    return true; 
}

// Thread to read temperatures and run peltier continuously
void control_temperature() {
    adc_init();
    adc_gpio_init(26);                 // enabling adc 0 on pin 26
    adc_set_temp_sensor_enabled(true); // reads internal pico temp...
    
    struct repeating_timer timer;
    add_repeating_timer_ms(-500, control_temperature_callback, NULL, &timer); // 0.5s interval, "-" indicates running in the background (core 1)
    while(true) {
        tight_loop_contents();
    }
}

/// USB Serial Communication | testing communication & duty
// void usb_serial() {
//     char line[32];
//     int  pos = 0;
//     int ch;
//     stdio_init_all();

//     while (true) {
//         
//         while (!stdio_usb_connected()) {
//             sleep_ms(100);  // Wait for USB connection
//         }
//         ch = getchar_timeout_us(100000);
//         if (ch != PICO_ERROR_TIMEOUT) {
//             switch (ch) {
//                 case '9': hb.drive = 0.9; break;
//                 case '8': hb.drive = 0.8; break;
//                 case '7': hb.drive = 0.7; break;
//                 case '6': hb.drive = 0.6; break;
//                 case '5': hb.drive = 0.5; break;
//                 case '4': hb.drive = 0.4; break;
//                 case '3': hb.drive = 0.3; break;
//                 case '2': hb.drive = 0.2; break;
//                 case '1': hb.drive = 0.1; break;
//                 case '0': hb.drive = 0.0; break;
//                 case '-': hb.drive = -hb.drive; break;
//             }
//             printf("%f: T=%5.2f C, T_target=%5.2f C, drive=%.2f\n", (float)hb.t_now, hb.T_now, hb.T_target, hb.drive);
//             while (getchar_timeout_us(0) >= 0) { }
//         }
//         sleep_ms(100);
//     }
// }

// /// USB serial comms data logger
void usb_serial_request_reply(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);             // un-buffer stdout

    while (!stdio_usb_connected()) tight_loop_contents();

    printf("Pico data logger ready. Send REQ to read one sample.\r\n");

    char line[16];      
    int  idx = 0;

    while (true) {
        int ch = getchar_timeout_us(100000);
        if (ch == PICO_ERROR_TIMEOUT) { 
            tight_loop_contents();
            continue;
        }

        if (ch == '\r' || ch == '\n') {           
            line[idx] = '\0';
            idx = 0;

            if (strcmp(line, "REQ") == 0) {
                // snapshot of hb written by core-1  
                HBridge snap;
                uint32_t ints = save_and_disable_interrupts();
                snap = hb;
                restore_interrupts(ints);

                // output epoch, temp, set-point, drive 
                printf("%lu,%.2f,%.2f,%.2f\r\n",
                       (unsigned long)snap.t_now,
                       snap.T_now, snap.T_target, snap.drive);
                
            } else if (strcmp(line, "END") == 0) {
                printf("Stopped recording.\r\n");
                break;        
                
            } else if (line[0] != '\0') {
                // printf("ERR: unknown cmd: %s\r\n", line);
                host_cmd_execute(line, &hb);                     // enables run-time commands
            }
            
        } else if (idx < (int)sizeof(line) - 1) { 
            line[idx++] = (char)ch;              
        }
    }
}

// Main function
int main() {
    float T_target=31.0; // C
    float t_target=10.0; // s
    float gain=0.2;      // max allowed drive | was 0.7

    hbridge_init(&hb, T_target, t_target, gain);
    multicore_launch_core1(control_temperature); // Launch temperature thread on core 1
    // usb_serial();                             // USB comms on core 0
    usb_serial_request_reply();                  // Handle USB communication on core 0
    return 0;
}
