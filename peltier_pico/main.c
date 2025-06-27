#include "read_temp.h"
#include "hbridge_peltier.h"
#include "runtime_cmd.h"
// === for ds18b20 thermistor ===
#include "onewire_library.h"
#include "onewire_library.pio.h"
#include "ds18b20.h"


// Global temperature and data variables
volatile HBridge hb;
volatile HBridge hb2;                          // peltier-2 
#define DS_PIN 22                              // DQ (data) pin for ds18b20 thermistor-1 (potentially thermistor-2 as well)
static uint64_t sensor1_rom, sensor2_rom;
OW ow;                                         // 1-wire bus obj

// bool control_temperature_callback(struct repeating_timer *t) {
//     /*callback for single thermistor*/
//     if (!hb.enabled) {
//         return true;
//     }
//     // hbridge_update_T(&hb, time(NULL), read_peltier_thermistor()); // old thermistor
//     // hbridge_update_T(&hb, time(NULL), read_ds18b20_celsius());    
//     hbridge_update_T(&hb, time(NULL), read_ds18b20_by_rom();           
//     hbridge_hysteresis_drive(&hb);
//     return true; 
// }

bool control_temperature_callback(struct repeating_tiemr *t) {
    if (!hb.enabled && !hb2.enabled) {
        return true;
    }
    
    // reads temp from each ds18b20 (conversion completed from prev cycle)
    float T1 = read_ds18b20_by_rom(sensor1_rom);
    float T2 = read_ds18b20_by_rom(sensor2_rom);
    
    time_t now = time(NULL);
    hbridge_update_T(&hb, now, T1);
    hbridge_update_T(&hb2, now, T2);
    
    hbridge_hysteresis_drive(&hb);
    hbridge_hysteresis_drive(&hb2);
    
    // starts new temp conversion on both sensors for next cycle
    ow_reset(&ow);
    ow_send(&ow, OW_SKIP_ROM);
    ow_send(&ow, DS18B20_CONVERT_T);
    return true;
    
}

// 1-wire rom search test + init temp conversions (for dual independent control loops)
void control_temperature() {
    uint offset = pio_add_program(pio0, &onewire_program);
    ow_init(&ow, pio0, offset, DS_PIN);         // initializing 1-wire bus
    
    // searching for thermistor ROM codes |  CAN REPLACE WITH HARD CODED ADDRESSES, example: uint64_t sensor1_rom=bitcodeULL;
    uint64_t rom_codes[2];
    int count = ow_romsearch(&ow, rom_codes, 2, OW_SEARCH_ROM);
    if (count >=2) {
        uint64_t = sensor1_rom = rom_codes[0];
        uint64_t = sensor2_rom = rom_codes[1];
    } else {
        printf("Err: only %d DS18B20 sensor(s) found on the bus.\n", count);
    }
    // end search for ROm codes
    
    // starting an initial temp conversion on all sensors
    ow_reset(&ow);
    ow_send(&ow, OW_SKIP_ROM);
    ow_send(&ow, DS18B20_CONVERT_T);
    
    // setting up a repeating timer to periodically read temps and control peltiers
    struct repeating_timer timer;
    add_repeating_timer_ms(-750, control_temperature_callback, NULL, &timer);
    while (true) {
        tight_loop_contents();
    }
    
}

// void control_temperature() {
//     /*for single ds18b20 thermistor*/
//     uint offset = pio_add_program(pio0, &onewire_program);
//     ow_init(&ow, pio0, offset, DS_PIN);
//     struct repeating_timer timer;
//     add_repeating_timer_ms(-750, control_temperature_callback, NULL, &timer);
//     while (true) {
//         tight_loop_contents();
//     }
// }

// // Thread to read temperatures and run peltier continuously ------- old thermistor + Pico ADC
// void control_temperature() {
//     adc_init();
//     adc_gpio_init(26);                 // enabling adc 0 on pin 26
//     adc_set_temp_sensor_enabled(true); // reads internal pico temp...
//     
//     struct repeating_timer timer;
//     add_repeating_timer_ms(-500, control_temperature_callback, NULL, &timer); // 0.5s interval, "-" indicates running in the background (core 1)
//     while(true) {
//         tight_loop_contents();
//     }
// }

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

/// USB serial comms data logger
void usb_serial_request_reply(void) {
    stdio_init_all();
    printf("USB serial online.\n");
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
                       (unsigned long) snap.t_now,
                       snap.T_now, snap.T_target, snap.drive);
                
            } else if (strcmp(line, "END") == 0) {
                printf("Stopped recording.\r\n");
                break;        
                
            } else if (line[0] != '\0') {
                // enables run-time commands
		host_cmd_execute(line, &hb); 
            }
            
        } else {
	    if (idx < (int)sizeof(line) - 1) { 
                line[idx++] = (char)ch;              
            } else {
		// printf("ERR: command too long.\r\n");
		idx = 0; // reset index if line is too long
	    }
	}
    }
}

int main() {
    // === Peltier-1 params ===
    float T_target=30.0;    // ˚C, front-end target
    float t_target=10.0;    // s
    float gain=0.2;         // max allowed drive | was 0.7
    
    // === Peltier-2 params ===
    float T_target2=32.0; // ˚C, noise source target
    
    // usb_serial_request_reply();                 // try this first to see if we get the "Send REQ" message upon opening serial
    
    // === init peltier-1 control ===
    hbridge_init(&hb,  T_target, t_target, gain);
    hb.channel =1;                                 // designate as channel 1
    
    // === init peltier-2 control ===
    hb2 = hb;                                      // copies all configs and states then changes them as not to call hbridge_init() twice
    hb2.T_target = T_target2;
    hb2.T_now  = hb2.T_target;
    hb2.T_prev = hb2.T_target;
    hb2.channel = 2;
    hb2.enabled= true; 
    hb2.active = true;
    // hbridge_init(&hb2, T_target2, t_target, gain); // Peltier-2 -- test
    
    multicore_launch_core1(control_temperature);   // Launch temperature thread on core 1
    // usb_serial();                               // USB comms on core 0
    usb_serial_request_reply();                    // Handle USB communication on core 0
    return 0;
}
