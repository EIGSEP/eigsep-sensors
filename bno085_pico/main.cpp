#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno08x.h"
#include "utils.h"

#define I2C_BAUDRATE 400000
#define IMU_ADDR 0x4A

BNO08x imu1;
BNO08x imu2;

void init_i2c_bus(i2c_inst_t *i2c, uint sda, uint scl) {
    i2c_init(i2c, I2C_BAUDRATE);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void enable_imu_features(BNO08x& imu) {
    imu.enableRotationVector();
    imu.enableAccelerometer();
    imu.enableLinearAccelerometer();
    imu.enableGyro();
    imu.enableMagnetometer();
    imu.enableGravity();
}

void print_sensor_data(BNO08x& imu, const char* label) {
    if (!imu.getSensorEvent()) return;

    sh2_SensorValue_t event = imu.sensorValue;

    printf("[%s] ", label);

    switch (event.sensorId) {
        case SENSOR_REPORTID_ROTATION_VECTOR:
            printf("q:%.3f:%.3f:%.3f:%.3f,",
                   event.un.rotationVector.i,
                   event.un.rotationVector.j,
                   event.un.rotationVector.k,
                   event.un.rotationVector.real);
            break;
        case SENSOR_REPORTID_ACCELEROMETER:
            printf("a:%.3f:%.3f:%.3f,", event.un.accelerometer.x,
                   event.un.accelerometer.y, event.un.accelerometer.z);
            break;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:
            printf("la:%.3f:%.3f:%.3f,", event.un.linearAcceleration.x,
                   event.un.linearAcceleration.y, event.un.linearAcceleration.z);
            break;
        case SENSOR_REPORTID_RAW_GYROSCOPE:
            printf("g:%.3f:%.3f:%.3f,", event.un.gyroscope.x,
                   event.un.gyroscope.y, event.un.gyroscope.z);
            break;
        case SENSOR_REPORTID_MAGNETIC_FIELD:
            printf("m:%.3f:%.3f:%.3f,", event.un.magneticField.x,
                   event.un.magneticField.y, event.un.magneticField.z);
            break;
        case SENSOR_REPORTID_GRAVITY:
            printf("grav:%.3f:%.3f:%.3f,", event.un.gravity.x,
                   event.un.gravity.y, event.un.gravity.z);
            break;
        default:
            printf("Unknown event ID: %d", event.sensorId);
    }

    printf("\n");
}

void calibrate_imu(BNO08x& imu) {
    printf("Starting calibration...\n");
    while (true) {
        if (imu.getSensorEvent()) {
            sh2_SensorValue_t event = imu.sensorValue;
            printf("Sensor %d accuracy: %d\n", event.sensorId, event.status);
            if (event.status >= 3) {
                printf("Sensor %d is calibrated.\n", event.sensorId);
                break;
            }
        }
        sleep_ms(250);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);  // USB startup delay

    init_i2c_bus(i2c0, 0, 1);
    init_i2c_bus(i2c1, 2, 3);

    while (!imu1.begin(IMU_ADDR, i2c0)) {
        printf("IMU1 not detected on i2c0\n");
        sleep_ms(1000);
    }
    while (!imu2.begin(IMU_ADDR, i2c1)) {
        printf("IMU2 not detected on i2c1\n");
        sleep_ms(1000);
    }

    enable_imu_features(imu1);
    enable_imu_features(imu2);

    char buf[16];
    while (true) {
        if (fgets(buf, sizeof(buf), stdin)) {
            if (strncmp(buf, "REQ", 3) == 0) {
                print_sensor_data(imu1, "IMU1");
                print_sensor_data(imu2, "IMU2");
            } else if (strncmp(buf, "CAL", 3) == 0) {
                calibrate_imu(imu1);
            }
        }
        sleep_ms(50);
    }
}
