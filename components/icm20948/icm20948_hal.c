//
// Created by Vitaliy on 19.11.2023.
//

/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Icm20948/Icm20948.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/SensorConfig.h"
#include "Invn/EmbUtils/InvScheduler.h"
#include "Invn/EmbUtils/RingByteBuffer.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/EmbUtils/RingBuffer.h"
//#include "Invn/DynamicProtocol/DynProtocol.h"
//#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#include "system.h"
#include "sensor.h"
#include "icm20948_hal.h"

#define SCHEDULER_PERIOD 1000	/* scheduler period in us */

static const char *TAG = "ICM_HAL";

void icm20948_hal_init(void )
{
    int rc = 0;
    /* Initialize External Sensor Interrupt */
    interface_initialize();

    /*
    * Initialize icm20948 serif structure
    */
    struct inv_icm20948_serif icm20948_serif;
    icm20948_serif.context   = 0; /* no need */
    icm20948_serif.read_reg  = idd_io_hal_read_reg;
    icm20948_serif.write_reg = idd_io_hal_write_reg;
    icm20948_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
    icm20948_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */

    icm20948_serif.is_spi = interface_is_SPI();

    /*
    * Reset icm20948 driver states
    */
    inv_icm20948_reset_states(&icm_device, &icm20948_serif);

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    /*
    * Setup the icm20948 device
    */
    rc = icm20948_sensor_setup();

    /*
     * inv_icm20948_set_sensor_period - set ODR (output data rate)
     */

    /*
    * Now that Icm20948 device was initialized, we can proceed with DMP image loading
    * This step is mandatory as DMP image are not store in non-volatile memory
    */
    rc += load_dmp3();
    check_rc(rc, "Error sensor_setup/DMP loading.");

//    ESP_LOGI(TAG, "Command START SENSOR INV_SENSOR_TYPE_ROTATION_VECTOR");
//    handle_command(DYN_PROTOCOL_EID_START_SENSOR, INV_SENSOR_TYPE_ROTATION_VECTOR);
//    ESP_LOGI(TAG, "Command START DONE");
    ESP_LOGI(TAG, "Command START SENSOR INV_SENSOR_TYPE_ROTATION_VECTOR");
    handle_command(DYN_PROTOCOL_EID_START_SENSOR, INV_SENSOR_TYPE_ACCELEROMETER);
    ESP_LOGI(TAG, "Command START DONE");
    ESP_LOGI(TAG, "Command START SENSOR INV_SENSOR_TYPE_ROTATION_VECTOR");
    handle_command(DYN_PROTOCOL_EID_START_SENSOR, INV_SENSOR_TYPE_GYROSCOPE);
    ESP_LOGI(TAG, "Command START DONE");
    ESP_LOGI(TAG, "Command START SENSOR INV_SENSOR_TYPE_ROTATION_VECTOR");
    handle_command(DYN_PROTOCOL_EID_START_SENSOR, INV_SENSOR_TYPE_MAGNETOMETER);
    ESP_LOGI(TAG, "Command START DONE");
    /*
     * Calibrated:
     * INV_SENSOR_TYPE_ACCELEROMETER
     * INV_SENSOR_TYPE_GYROSCOPE
     * INV_SENSOR_TYPE_MAGNETOMETER
     *
     * Raw:
     * INV_SENSOR_TYPE_UNCAL_GYROSCOPE
     * INV_SENSOR_TYPE_UNCAL_MAGNETOMETER
     *
     */
}

void icm20948_hal_poll(void )
{
    inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);
}

void icm20948_hal_print() {
    ESP_LOGI(TAG, "ICM LP Mode: %u", icm_device.base_state.chip_lp_ln_mode);
}

///** @brief Hook for low-level system sleep() function to be implemented by upper layer
// *  @param[in] ms number of millisecond the calling thread should sleep
// */
//void inv_icm20948_sleep_us(int us) {
//
//}
//
///** @brief Hook for low-level system time() function to be implemented by upper layer
// *  @return monotonic timestamp in us
// */
//uint64_t inv_icm20948_get_time_us(void) {
//
//}