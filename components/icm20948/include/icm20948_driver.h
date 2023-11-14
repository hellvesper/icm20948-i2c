//
// icm20948 i2c driver by hellvesper
//

#ifndef I2CSCANNER_ICM20948_DRIVER_H
#define I2CSCANNER_ICM20948_DRIVER_H

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

esp_err_t write_icm_register(uint8_t slv_addr, uint8_t reg_addr, uint8_t data);

esp_err_t write_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, const uint8_t *data, uint8_t data_len);

esp_err_t read_icm_register(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data);

esp_err_t read_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len);



#endif //I2CSCANNER_ICM20948_DRIVER_H
