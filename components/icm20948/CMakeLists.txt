set(incs "include")
set(priv_incs
 "."
 "Invn"
 "Invn/Common"
 "Invn/Devices"
 "Invn/EmbUtils"
 "Invn/Images"
 "Invn/VSensor"
 "Invn/Devices/Drivers/ICM20948"
 "Invn/Devices/Drivers/Ak0991x"
)
set(srcs "system.c"
 "icm20948_driver.c"
 "icm20948_hal.c"
 "sensor.c"
 "Invn/EmbUtils/InvProtocol.c"
 "Invn/EmbUtils/InvScheduler.c"
 "Invn/EmbUtils/InvQueue.c"
 "Invn/EmbUtils/DataConverter.c"
 "Invn/EmbUtils/RingByteBuffer.c"
 "Invn/EmbUtils/ErrorHelper.c"
 "Invn/EmbUtils/Message.c"
 "Invn/Devices/Sensor.c"
 "Invn/Devices/Drivers/Ak0991x/Ak0991x.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948LoadFirmware.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948Setup.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948Dmp3Driver.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948DataBaseDriver.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948Augmented.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948AuxTransport.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948AuxCompassAkm.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948SelfTest.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948DataConverter.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948MPUFifoControl.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948Transport.c"
 "Invn/Devices/Drivers/ICM20948/Icm20948DataBaseControl.c"
) 

idf_component_register(SRCS ${srcs}
        INCLUDE_DIRS ${incs}
        PRIV_INCLUDE_DIRS ${priv_incs})
