//
// imc20948 i2c idf driver by hellvesper
//

#include "icm20948_driver.h"

#define ACK_EN 1

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