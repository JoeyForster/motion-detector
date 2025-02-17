#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// I2C configuration
#define I2C_MASTER_SCL_IO 20
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68

// Function to initialize the MPU6050
void initMPU6050() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // Power management register
    i2c_master_write_byte(cmd, 0, true); // Set to 0 to wake up MPU6050
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // Start I2C communication
    i2c_cmd_link_delete(cmd);
}

// Function to read the raw IMU data
void readIMU(void *pvParameters) {
    uint8_t data[14]; // Store the 14-byte data from the MPU6050
    float ax, ay, az;

    while(1) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x3B, true); // Start register for acceleration data
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK); // Read 14 bytes of data
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        // Convert raw accelerometer data to float values (scaled to ±2g)
        ax = (int16_t)(data[0] << 8 | data[1]) / 16384.0; // Normalize to ±2g
        ay = (int16_t)(data[2] << 8 | data[3]) / 16384.0;
        az = (int16_t)(data[4] << 8 | data[5]) / 16384.0;

        // Print the raw accelerometer values to the serial terminal
        printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n", ax, ay, az);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms (10Hz frequency)
    }
}

void app_main(void) {
    // Initialize I2C and the MPU 6050
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_MASTER_SDA_IO;
    config.scl_io_num = I2C_MASTER_SCL_IO;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &config);
    i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);

    // Initialize the MPU6050
    initMPU6050();

    // Create the task to read and print the IMU data
    xTaskCreate(&readIMU, "readIMU", 2048, NULL, 5, NULL);
}
