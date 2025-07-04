#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_FREQ 100000
#define I2C_ADDR 0x66

#define CMD_OUTPUT_CONFIG 27
#define CMD_READ 44
#define CMD_UPDATE_RATE 74

#define MAX_RATE 50
#define OUTPUT_FLAGS 0b01100010
#define LOOP_DELAY_MS 50

void init_i2c() {
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void write_u32(uint8_t cmd, uint32_t value) {
    uint8_t payload[5];
    payload[0] = cmd;
    payload[1] = value & 0xFF;
    payload[2] = (value >> 8) & 0xFF;
    payload[3] = (value >> 16) & 0xFF;
    payload[4] = (value >> 24) & 0xFF;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, payload, 5, false);
}

uint32_t read_u32(uint8_t cmd) {
    uint8_t data[4];
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &cmd, 1, false);
    sleep_ms(20);
    i2c_read_blocking(I2C_PORT, I2C_ADDR, data, 4, false);
    return (uint32_t)data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

int main() {
    stdio_init_all();
    init_i2c();
    sleep_ms(1000);  // allow USB connection

    // Wait for device to respond
    uint8_t found = 0;
    while (!found) {
        uint8_t dummy = 0;
        found = i2c_write_blocking(I2C_PORT, I2C_ADDR, &dummy, 1, true) >= 0;
        if (!found) sleep_ms(100);
    }
    printf("Found GRF-250 at 0x%02X\n", I2C_ADDR);

    // Configure update rate and output
    write_u32(CMD_UPDATE_RATE, MAX_RATE);
    sleep_ms(50);
    uint32_t rate = read_u32(CMD_UPDATE_RATE);

    write_u32(CMD_OUTPUT_CONFIG, OUTPUT_FLAGS);
    sleep_ms(50);
    uint32_t flags = read_u32(CMD_OUTPUT_CONFIG);

    // Main loop
    char buf[16];
    while (true) {
        if (fgets(buf, sizeof(buf), stdin)) {
            if (strncmp(buf, "REQ", 3) == 0) {
                uint8_t cmd = CMD_READ;
                i2c_write_blocking(I2C_PORT, I2C_ADDR, &cmd, 1, false);
                sleep_ms(20);
                uint8_t resp[12];
                int read = i2c_read_blocking(I2C_PORT, I2C_ADDR, resp, 12, false);

                if (read == 12) {
                    int32_t dist_raw = (int32_t)(resp[0] | (resp[1] << 8) | (resp[2] << 16) | (resp[3] << 24));
                    int32_t strength = (int32_t)(resp[4] | (resp[5] << 8) | (resp[6] << 16) | (resp[7] << 24));
                    int32_t temp_raw = (int32_t)(resp[8] | (resp[9] << 8) | (resp[10] << 16) | (resp[11] << 24));
                    printf("%.2f,%d,%.2f\n", dist_raw / 10.0, strength, temp_raw / 100.0);
                } else {
                    printf("Unexpected response length: %d\n", read);
                }
            }
        }
        sleep_ms(LOOP_DELAY_MS);
    }

    return 0;
}
