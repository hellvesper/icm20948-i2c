#include <driver/i2c.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

#include "icm20948_driver.h"
#include "icm20948_hal.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SDA_PIN 41
#define SCL_PIN 42
#define ICM_INT 40

#define ACK_EN 1
#define NACK 0
#define ICM20948_ADDR (0x68)
#define ICM_WHO_AM_I_REG (0x00)

#define BUF_SIZE (1024)
#define ABORT_K (0x03)
#define RETURN_K (0x0d)
#define READ_REG 1
#define WRITE_REG 2
static const char *TAG = "icm20948-i2c";

enum i2c_states {
    RW_DIR,
    AWAIT_RW_DIR,
    REGW,
    AWAIT_REGW,
    DATAW,
    AWAIT_DATAW,
    PROCESSING_CMD,
    CH_SLV_ADDR,
    AWAIT_CH_SLV_ADDR,
    I2C_SCAN
};

struct i2c_action {
    uint8_t slv_addr;
    uint8_t dir;
    uint8_t reg;
    uint8_t data;
};

int state = 0;
xQueueHandle interputQueue;

static void IRAM_ATTR icm_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
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
    uint8_t data_byte = 0;
    uint8_t data_arr[10];
    esp_err_t res = ESP_FAIL;
    while (1)
    {
        search_devices();
//        res = read_icm_register(ICM20948_ADDR, ICM_WHO_AM_I_REG, &data_byte);
        res = read_icm_register_burst(ICM20948_ADDR, ICM_WHO_AM_I_REG, data_arr, 10);
        if (res == ESP_OK) {
            printf("reg: 0x%02X content: 0x%02X\n", 0x00, data_byte);
            for (int i = 0; i < 10; ++i) {
                printf("reg: 0x%02X content: 0x%02X\n", ICM_WHO_AM_I_REG + i, data_arr[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t hexStringToUint8(const uint8_t* hexString, uint8_t *data) {
    long result = strtol((const char *)hexString, NULL, 16);

    if (result < 0 || result > UINT8_MAX) {
        return ESP_FAIL;
    }

    *data = (uint8_t)result;

    return ESP_OK;
}

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_43, GPIO_NUM_44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    // Init state machine
    static enum i2c_states currentState = RW_DIR;
    static struct i2c_action i2c_cmd;
    // set ICM by default
    i2c_cmd.slv_addr = ICM20948_ADDR;

    while (1) {
        int len;
        uint8_t parsed_value;
        esp_err_t res = ESP_FAIL;

        switch (currentState) {

            case RW_DIR:
                ESP_LOGI(TAG, "SLV: 0x%x | Enter direction 1 - Read. 2 - write, 3 - change slave, 4 - scan i2c bus", i2c_cmd.slv_addr);
                currentState++;
                break;
            case AWAIT_RW_DIR:
                len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1), 1000 / portTICK_PERIOD_MS);
                if (len && data[0] == ABORT_K) {
                    currentState = RW_DIR;
                    break;
                }
                if (len == 1) {
                    data[len] = '\0';
                    if (hexStringToUint8(data, &parsed_value) == ESP_OK) {
                        if (parsed_value == 3) {
                            currentState = CH_SLV_ADDR;
                            break;
                        } else if (parsed_value == 4) {
                            currentState = I2C_SCAN;
                            break;
                        } else if (parsed_value == 1 || parsed_value == 2) {
                            i2c_cmd.dir = parsed_value;
                            currentState++;
                        } else
                        {
                            ESP_LOGE(TAG, "Direction should be 1(R) or 2(W) got %x (%x)", parsed_value, data[0]);
                        }
                    } else
                    {
                        ESP_LOGE(TAG, "Parsing error, raw value %s: parsed value: %x", data, parsed_value);
                    }
                }
                break;
            case REGW:
                ESP_LOGI(TAG, "Enter register number between  00 and 7F");
                currentState++;
                break;
            case AWAIT_REGW:
                // Read data from the UART
                len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1), 1000 / portTICK_PERIOD_MS);
                if (len && data[0] == ABORT_K) {
                    currentState = RW_DIR;
                    break;
                }
                if (len == 2) {
                    data[len] = '\0';
                    if (hexStringToUint8(data, &parsed_value) == ESP_OK) {
                        if (parsed_value <= 0x7F) {
                            i2c_cmd.reg = parsed_value;
                            if (i2c_cmd.dir == 2) {
                                // write to register
                                currentState++;
                            } else {
                                // read from register, data isn't needed
                                currentState = PROCESSING_CMD;
                            }
                        }
                        else {
                            ESP_LOGE(TAG, "Register num should be lower or equal 0x7F got %x", parsed_value);
                        }
                    } else
                    {
                        ESP_LOGE(TAG, "Parsing error, raw value %s: parsed value: %x", data, parsed_value);
                    }
                }
                break;
            case DATAW:
                ESP_LOGI(TAG, "Enter 1 byte DATA value 00 - FF");
                currentState++;
                break;
            case AWAIT_DATAW:
                len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1), 1000 / portTICK_PERIOD_MS);
                if (len && data[0] == ABORT_K) {
                    currentState = RW_DIR;
                    break;
                }
                if (len == 2) {
                    data[len] = '\0';
                    if (hexStringToUint8(data, &parsed_value) == ESP_OK) {
                        if (parsed_value <= 0xFF) {
                            i2c_cmd.data = parsed_value;
                            currentState++;
                        }
                        else {
                            ESP_LOGE(TAG, "Data should be 1 byte long in 00 - FF got %x", parsed_value);
                        }
                    } else
                    {
                        ESP_LOGE(TAG, "Parsing error, raw value %s: parsed value: %x", data, parsed_value);
                    }
                }
                break;
            case PROCESSING_CMD:
                if (i2c_cmd.dir == READ_REG) {
                    res = read_icm_register(i2c_cmd.slv_addr, i2c_cmd.reg, &i2c_cmd.data);
                    if (res == ESP_OK) {
                        printf("reg: 0x%02X content: 0x%02X\n", i2c_cmd.reg, i2c_cmd.data);
                        currentState = RW_DIR;
                    } else {
                        ESP_LOGE(TAG, "Can't read register, something goes wrong");
                        currentState = RW_DIR;
                    }
                } else if (i2c_cmd.dir == WRITE_REG) {
                    res = write_icm_register(i2c_cmd.slv_addr, i2c_cmd.reg, &i2c_cmd.data);
                    if (res == ESP_OK) {
                        printf("reg: 0x%02X written with content: 0x%02X\n", i2c_cmd.reg, i2c_cmd.data);
                        currentState = RW_DIR;
                    } else {
                        ESP_LOGE(TAG, "Can't write register, something goes wrong %d", res);
                        currentState = RW_DIR;
                    }
                }
                break;
            case CH_SLV_ADDR:
                ESP_LOGI(TAG, "Enter new slave address 00 - 7F");
                currentState++;
                break;
            case AWAIT_CH_SLV_ADDR:
                // Read data from the UART
                len = uart_read_bytes(UART_NUM_0, data, (BUF_SIZE - 1), 1000 / portTICK_PERIOD_MS);
                if (len && data[0] == ABORT_K) {
                    currentState = RW_DIR;
                    break;
                }
                if (len == 2) {
                    data[len] = '\0';
                    if (hexStringToUint8(data, &parsed_value) == ESP_OK) {
                        if (parsed_value <= 0x7F) {
                            i2c_cmd.slv_addr = parsed_value;
                            // reset state machine
                            currentState = RW_DIR;
                        }
                        else {
                            ESP_LOGE(TAG, "Slave ADDR num should be lower or equal 0x7F got %x", parsed_value);
                        }
                    } else
                    {
                        ESP_LOGE(TAG, "Parsing error, raw value %s: parsed value: %x", data, parsed_value);
                    }
                }
                break;
            case I2C_SCAN:
                search_devices();
                currentState = RW_DIR;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void icm_task(void *arg) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
            .baud_rate = 921600, // / 8, // 115200
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            // .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
            .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_43, GPIO_NUM_44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Init ICM20948");
    icm20948_hal_init();
    ESP_LOGI(TAG, "Init Done");
    ESP_LOGI(TAG, "Start Polling");
    // static uint8_t ubyte[]= {0x14, 0x88};
    // static uint32_t count = 0;
    while (1) {
        icm20948_hal_poll();

        // int res = uart_write_bytes(UART_NUM_0, &ubyte, 2);
        // Write data to UART.
        // char* test_str = "AAAA\r\n";
        // char* test_str2 = "BBBB\n";
        // if (count % 2 == 0) {
        //     int res = uart_write_bytes(UART_NUM_0, (const char*)test_str, strlen(test_str));
        //
        // } else {
        //
        //     printf(test_str2);
        // }
        // count++;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (1) {
        icm20948_hal_poll();
        uart_write_bytes(UART_NUM_0, "hello", sizeof("hello"));
#define DEBUG_PRINT 1
#ifdef DEBUG_PRINT
        static uint32_t print_time = 0;
        if (print_time % 1000 == 0) {
//            icm20948_hal_print();
            ESP_LOGI(TAG, "Poll cont: %u", print_time);
            char buffer[1024];
            vTaskGetRunTimeStats(buffer);
            printf("\n\n%s\n", buffer);
        }
        print_time++;
#endif
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void LED_Control_Task(void *params)
{
    int pinNumber, count = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count++, gpio_get_level(ICM_INT));
//            gpio_set_level(LED_PIN, gpio_get_level(ICM_INT));

        }
    }
}


void app_main()
{
    gpio_pad_select_gpio(ICM_INT);
    gpio_set_direction(ICM_INT, GPIO_MODE_INPUT);
    gpio_pulldown_en(ICM_INT);
    gpio_pullup_dis(ICM_INT);
    gpio_set_intr_type(ICM_INT, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ICM_INT, icm_interrupt_handler, (void *)ICM_INT);

    if (init_i2c() != ESP_OK) {
        ESP_LOGE(TAG, "Can't init i2c bus");
        while (1); // hang esp
    }
    // Start task
//    xTaskCreatePinnedToCore(task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, PRO_CPU_NUM);
//    xTaskCreatePinnedToCore(echo_task, "uart_echo_task", 2048, NULL, 10, NULL, PRO_CPU_NUM);
    xTaskCreatePinnedToCore(icm_task, "ICM_TASK", 2048 * 2, NULL, 10, NULL, PRO_CPU_NUM);
//    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}
