#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 21    // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO 22    // GPIO pin for I2C data
#define I2C_MASTER_FREQ_HZ 100000   // I2C master clock frequency
#define MMC5983MA_ADDR 0x30    // MMC5983MA device address
#define REGISTER_ADDRESS 0x03  // Address of the data to be read

i2c_config_t conf;

bool I2C_master_init(i2c_config_t *i2c, i2c_port_t port, uint8_t sda, uint8_t scl, uint32_t freq_hz) {
    esp_err_t res;
    i2c->mode = I2C_MODE_MASTER;
    i2c->sda_io_num = sda;
    i2c->scl_io_num = scl;
    i2c->sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c->scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c->master.clk_speed = freq_hz;
    i2c->clk_flags = 0;

    res = i2c_param_config(port, i2c);
    res = i2c_driver_install(port, i2c->mode, 0, 0, 0);
    return res == ESP_OK;
}


esp_err_t read_sensor(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MMC5983MA_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REGISTER_ADDRESS, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MMC5983MA_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void app_main(void) {
    if(I2C_master_init(&conf, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ)) {
        printf("I2C initialized\n");
    } else {
        printf("I2C initialization failed\n");
    }

    while (1) {
        uint8_t data[8];
        esp_err_t res = read_sensor(data, 8);
        if (res == ESP_OK) {
            printf("Data read: ");
            for (int i = 0; i < 8; i++) {
                printf("%d ", data[i]);
            }
            printf("\n");
        } else {
            printf("Data read failed\n");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Oczekiwanie 1 sekundy
    }
}