//
// imc20948 i2c idf driver by hellvesper
//

#include "icm20948_driver.h"
//#include "HAL.h"
//#include "ICM20948.h"
#include "icm20948_hal.h"

#define ACK_EN 1
#define SDA_PIN 41
#define SCL_PIN 42

static SemaphoreHandle_t i2c_semaphore = NULL;

esp_err_t init_i2c(void)
{
    // Don't initialize twice
    if(i2c_semaphore != NULL)
        return ESP_FAIL;

    i2c_semaphore = xSemaphoreCreateMutex();
    if(i2c_semaphore == NULL)
        return ESP_FAIL;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = I2C_SCLK_DEFAULT;

    esp_err_t ret;

    ret = i2c_param_config(I2C_NUM_0, &conf);
    if(ret != ESP_OK)
        return ret;

    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if(ret != ESP_OK)
        return ret;

    return ESP_OK;
}


esp_err_t write_icm_register(uint8_t slv_addr, uint8_t reg_addr, const uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit register
    i2c_master_write_byte(cmd, reg_addr, ACK_EN);
    // transmit data
    i2c_master_write_byte(cmd, *data, ACK_EN);
//    i2c_master_write(cmd,)
    i2c_master_stop(cmd);
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    xSemaphoreGive(i2c_semaphore);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t write_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, const uint8_t *data, uint8_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit register
    i2c_master_write_byte(cmd, reg_addr, ACK_EN);
    // transmit data
    i2c_master_write(cmd, data, data_len, ACK_EN);
    i2c_master_stop(cmd);
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    xSemaphoreGive(i2c_semaphore);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t read_icm_register(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit reg to read
    i2c_master_write_byte(cmd,reg_addr, ACK_EN /* expect ack */);
    // receive data sequence
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_READ, ACK_EN /* expect ack */);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    xSemaphoreGive(i2c_semaphore);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t read_icm_register_burst(uint8_t slv_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_OK;
    // transmit addr
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
    // transmit reg to read
    i2c_master_write_byte(cmd,reg_addr, ACK_EN /* expect ack */);
    // recieve data
    // repeat start sequence with addr and transfer direction
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slv_addr << 1) | I2C_MASTER_READ, ACK_EN /* expect ack */);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    xSemaphoreGive(i2c_semaphore);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void search_devices()
{
    esp_err_t ret;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t i = 3; i < 0x78; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, ACK_EN /* expect ack */);
        i2c_master_stop(cmd);

        xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        xSemaphoreGive(i2c_semaphore);
        if (i % 16 == 0)
            printf("\n%.2x:", i);
        if (ret == 0)
            printf(" %.2x", i);
        else
            printf(" --");
        i2c_cmd_link_delete(cmd);
    }
    printf("\n\n");
}