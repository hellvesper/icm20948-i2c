/*
* ________________________________________________________________________________________________________
* Copyright (c) 2017 InvenSense Inc. All rights reserved.
*
* This software, related documentation and any modifications thereto (collectively �Software�) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
* and other intellectual property rights laws.
*
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license agreement
* from InvenSense is strictly prohibited.
*
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
* PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
* TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
* EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
* INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
* DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
* NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
* OF THE SOFTWARE.
* ________________________________________________________________________________________________________
*/
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

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
#include "icm20948_driver.h"

#if SERIF_TYPE_SPI
static int spi_master_transfer_rx(void * context, uint8_t register_addr, uint8_t * value, uint32_t len);
static int spi_master_transfer_tx(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len);
#endif
#if SERIF_TYPE_I2C
static void i2c_master_initialize(void);
static unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
static unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
#endif

extern volatile int irq_from_device;
#if SERIF_TYPE_I2C
/*
* Variable for storing I2C Address
*/
uint8_t I2C_Address = ICM_I2C_ADDR_REVB;
#endif

#if SERIF_TYPE_I2C

static void i2c_master_initialize(void)
{
    init_i2c();
}
static unsigned long i2c_master_read_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
    if (read_icm_register_burst(Address, RegisterAddr, RegisterValue, RegisterLen) == ESP_OK) {
        return TWI_SUCCESS;
    } else {
        return TWI_BUSY;
    }
}

static unsigned long i2c_master_write_register(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
    if (write_icm_register_burst(Address, RegisterAddr, RegisterValue, RegisterLen) == ESP_OK) {
        return TWI_SUCCESS;
    } else {
        return TWI_RECEIVE_NACK;
    }
}

#endif

#if SERIF_TYPE_SPI
#define SPI_Handler				FLEXCOM5_Handler
#define SPI_IRQn				FLEXCOM5_IRQn
#define SPI_CHIP_SEL			CHIP_SELECT /* Chip select. */
#define SPI_CHIP_PCS			spi_get_pcs(SPI_CHIP_SEL)

/*
* Mode 0: POL=0, PHA=1
* Mode 1: POL=0, PHA=0
* Mode 2: POL=1, PHA=1
* Mode 3: POL=1, PHA=0
*/
#define SPI_CLK_POLARITY		0		/* Clock polarity. */
#define SPI_CLK_PHASE			1		/* Clock phase. */
#define SPI_DLYBS				0x40	/* Delay before SPCK. */
#define SPI_DLYBCT				0x10	/* Delay between consecutive transfers. */
#define SPI_CLK_SPEED			1562000	/* SPI clock setting (Hz). */

#define READ_BIT_MASK			0x80
#define WRITE_BIT_MASK			0x7F

/* Function prototype declaration */
/**
* \brief Initialize SPI as master.
*/
void spi_master_initialize(void){
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI, FLEXCOM_SPI);

	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);

	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_SEL);

	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL, (sysclk_get_peripheral_hz() / SPI_CLK_SPEED));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS, SPI_DLYBCT);

	spi_enable(SPI_MASTER_BASE);
}

static int spi_master_transfer_tx(void * context, uint8_t register_addr, const uint8_t * value, uint32_t len){
	uint8_t reg	= register_addr; 
	const uint8_t *p_rbuf = value; 
	uint32_t rsize = len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;

	delay_us(1);
	reg &= WRITE_BIT_MASK;

	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, p_rbuf[i], 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
	}

	return 0;
}

static int spi_master_transfer_rx(void * context, uint8_t register_addr, uint8_t * value, uint32_t len){
	uint8_t reg	= register_addr; 
	uint8_t *p_rbuf	= value; 
	uint32_t rsize	= len;
	uint32_t i;
	uint8_t uc_pcs = 0;
	uint16_t data = 0;

	delay_us(1);
	reg |= READ_BIT_MASK;

	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */

	for (i = 0; i < rsize; i++) {
		spi_write(SPI_MASTER_BASE, 0x0, 0, 0); /* dummy write to generate clock */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
		p_rbuf[i] = (uint8_t)(data & 0xFF);
	}

	return 0;
}
#endif

inv_bool_t interface_is_SPI(void){
#if SERIF_TYPE_SPI
	return true;
#else
	return false;
#endif	
}
void interface_initialize(void){
#if SERIF_TYPE_SPI
	spi_master_initialize();
#endif

#if SERIF_TYPE_I2C
	i2c_master_initialize();
#endif
}

void switch_I2C_to_revA(void){
#if SERIF_TYPE_I2C
	I2C_Address = ICM_I2C_ADDR_REVA;
#endif	
}

InvScheduler 	scheduler;

//
//}

//void hw_timer_stop(void){
//	tc_stop(TC0, 0);
//	NVIC_DisableIRQ(TC0_IRQn);
//	NVIC_ClearPendingIRQ(TC0_IRQn);
//}

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen){
	(void)context;
#if SERIF_TYPE_SPI
	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
#else /* SERIF_TYPE_I2C */
	return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
#endif	
}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen){
	(void)context;
#if SERIF_TYPE_SPI
	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);
#else /* SERIF_TYPE_I2C */
	return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
#endif	
}
