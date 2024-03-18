#include <stdio.h>
#include <driver/i2c.h>
#include "esp_log.h"
#define TAG "I2C"

typedef i2c_config_t i2c_t;

bool I2C_master_init(i2c_t *i2c, i2c_port_t port, uint8_t sda, uint8_t scl, uint32_t freq_hz) {
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
bool I2C_master_read(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t res;
    uint8_t buffer[200] = { 0 };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert (cmd != NULL);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    res = i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "CHUJ");
        return false;
    }
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));

    i2c_cmd_link_delete(cmd);

    return res == ESP_OK;
}

bool I2C_master_write(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len) {
    esp_err_t res;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));

    i2c_cmd_link_delete(cmd);

    return res == ESP_OK;
}
void app_main(void)
{
    uint8_t date[6];
    uint8_t cmd = 0x01;
    i2c_t i2c;
    if(!I2C_master_init(&i2c,I2C_NUM_0,22,21,40000)){
        ESP_LOGE(TAG,"I2C INIT ERROR");
    }
    while(1){

        if(!I2C_master_write(I2C_NUM_0,0x30,0x09,&cmd,1)){
            ESP_LOGE(TAG,"XD");
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
        if(!I2C_master_read(I2C_NUM_0,0x30,0x00,date,6)){
            ESP_LOGE(TAG,"DEVICE ADDRES READ ERROR");
        }
        float x = 0;
        uint16_t resultx = 0; 
        resultx |= date[1];
        resultx |= date[0] << 8;
        x = (float)(resultx - 32768.0);
        x = x / 4096.0;

        float y = 0;
        uint16_t resulty = 0;
        resulty |= date[3];
        resulty |= date[2] << 8;
        y = (float)(resulty - 32768.0);
        y = y / 4096.0;

        float z = 0;
        uint16_t resultz = 0;
        resultz |= date[5];
        resultz |= date[4] << 8;
        z = (float)(resultz - 32768.0);
        z = z / 4096.0;


        ESP_LOGI(TAG,"x = %f y = %f z = %f",x, y ,z);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}


