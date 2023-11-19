//
// Created by Vitaliy on 19.11.2023.
//

#include "icm20948_hal.h"

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
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"


#include "system.h"
#include "sensor.h"

#define SCHEDULER_PERIOD 1000	/* scheduler period in us */
/** @brief ICM20948 serial interface
 */
struct inv_icm20948_serif {
    void *     context;
    int      (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len);
    int      (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len);
    uint32_t   max_read;
    uint32_t   max_write;
    inv_bool_t is_spi;
};

struct inv_icm20948_serif icm_serf;
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