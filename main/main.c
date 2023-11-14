#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "icm20948_driver.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SDA_PIN 41
#define SCL_PIN 42

#define ACK_EN 1
#define NACK 0

static const char *TAG = "icm20948-i2c";

void search_devices()
{
    esp_err_t res;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t i = 3; i < 0x78; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (i % 16 == 0)
            printf("\n%.2x:", i);
        if (res == 0)
            printf(" %.2x", i);
        else
            printf(" --");
        i2c_cmd_link_delete(cmd);
    }
    printf("\n\n");
}

esp_err_t write_icm_register(uint8_t slv_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (reg_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit data
    i2c_master_write_byte(cmd, data, ACK_EN);
//    i2c_master_write(cmd,)
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t write_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, const uint8_t *data, uint8_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (reg_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit data
//    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, data_len, ACK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t read_icm_register(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (reg_addr << 1) | I2C_MASTER_READ, ACK_EN /* expect ack */);
    // recieve data
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t read_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (reg_addr << 1) | I2C_MASTER_READ, ACK_EN /* expect ack */);
    // recieve data
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void task(void *ignore)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    while (1)
    {
        search_devices();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // Start task
    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
