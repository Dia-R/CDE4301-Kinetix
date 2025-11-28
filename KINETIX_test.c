// Insert Libraries Here
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"

// Insert definitions here
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_BAUDRATE 100000
#define ADXL345_ADDR 0x53
#define TCA9548A_ADDR 0x70

// Insert Init Func Here 
void my_i2c_init(){
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

int8_t i2c_write_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len){
    uint8_t buf[len + 1];
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }
    return i2c_write_blocking(I2C_PORT, addr, buf, len + 1, false);
}

int8_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len){
    int ret = i2c_write_blocking(I2C_PORT, addr, &reg, 1, true); // write register address
    if (ret < 0) return ret;
    ret = i2c_read_blocking(I2C_PORT, addr, data, len, false); // read data
    return ret;
}

void adxl345_init() {
    uint8_t data;

    // Reset offsets & power control
    data = 0x00;
    i2c_write_bytes(ADXL345_ADDR, 0x2D, &data, 1);  
    sleep_ms(5);

    // Enable measurement mode
    data = 0x08;
    i2c_write_bytes(ADXL345_ADDR, 0x2D, &data, 1);

    // ±2g range, full resolution OFF
    data = 0x00;
    i2c_write_bytes(ADXL345_ADDR, 0x31, &data, 1);

    // Disable FIFO (prevent stuck output = -512)
    data = 0x00;
    i2c_write_bytes(ADXL345_ADDR, 0x38, &data, 1);

    // 100 Hz data rate
    data = 0x0A;
    i2c_write_bytes(ADXL345_ADDR, 0x2C, &data, 1);
}

// Insert other functions here
void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t raw[6];
    int ret = i2c_read_bytes(ADXL345_ADDR, 0x32, raw, 6);

    if (ret < 0) {
    *ax = *ay = *az = -9999;
    return;
    }

    *ax = (int16_t)((raw[1] << 8) | raw[0]);
    *ay = (int16_t)((raw[3] << 8) | raw[2]);
    *az = (int16_t)((raw[5] << 8) | raw[4]);
}

void tca_select(uint8_t channel) {
    if (channel > 7) return;
    uint8_t data = 1 << channel;
    i2c_write_blocking(I2C_PORT, TCA9548A_ADDR, &data, 1, false);
}

void scan_channel(uint8_t channel) {
    tca_select(channel);
    for (uint8_t addr = 0x10; addr < 127; addr++) {
        uint8_t data;
        if (i2c_read_blocking(I2C_PORT, addr, &data, 1, false) >= 0) {
            printf(" - Found device at 0x%02X on channel %d\n", addr, channel);
        }
    }
}

// int main()
// {
//     // Initialise
//     stdio_init_all();
//     sleep_ms(2000); // Wait for USB Serial to initialize

//     // Init I2C
//     my_i2c_init();

//     for (uint8_t ch = 6; ch <= 7; ch++){
//         scan_channel(ch);
//     }

//     // Init both IMUS
//     tca_select(6);
//     sleep_ms(200);
//     adxl345_init();

//     tca_select(7);
//     sleep_ms(20);
//     adxl345_init();

//     int16_t ax, ay, az;
//     uint8_t curr_imu = 6; // Start with IMU 1 (Ch 6)
//     uint32_t last_swap = to_ms_since_boot(get_absolute_time());

//     while (true) {
//         // Switch channel
//         tca_select(curr_imu);

//         // Read accel values
//         read_accel(&ax, &ay, &az);
//         printf("IMU %d - Accel X: %d, Y: %d, Z: %d\n", curr_imu == 6 ? 1 : 2, ax, ay, az);

//         sleep_ms(1000);

//         if(to_ms_since_boot(get_absolute_time()) - last_swap >= 10000){
//             curr_imu = (curr_imu == 6) ? 7 : 6;
//             last_swap = to_ms_since_boot(get_absolute_time());
//         }
//     }
// }

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("BOOT OK\n");
    while (1) { sleep_ms(1000); printf("RUNNING\n"); }
}