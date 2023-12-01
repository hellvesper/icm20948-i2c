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
#define DEF_ST_ACCEL_FS                 2
#define DEF_ST_GYRO_FS_DPS              250
#define DEF_ST_SCALE                    32768
#define DEF_SELFTEST_GYRO_SENS			(DEF_ST_SCALE / DEF_ST_GYRO_FS_DPS)
#define DEF_ST_ACCEL_FS_MG				2000

#ifndef INV20948_ABS
#define INV20948_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

/** @brief Event identifier definition
 */
/* Commented label are currently not implemented */
/* Ported from DynProtocol.h for handle_command() */
enum DynProtocolEid {
    /* Protocol */
    DYN_PROTOCOL_EID_PROTOCOLVERSION = 0x00,

    /* IDD methods */
    DYN_PROTOCOL_EID_WHO_AM_I           = 0x10,
    DYN_PROTOCOL_EID_RESET              = 0x11,
    DYN_PROTOCOL_EID_SETUP              = 0x12,
    DYN_PROTOCOL_EID_CLEANUP            = 0x13,
    // DYN_PROTOCOL_EID_LOAD               = 0x14,
    DYN_PROTOCOL_EID_SELF_TEST          = 0x15,
    DYN_PROTOCOL_EID_GET_FW_INFO        = 0x16,
    DYN_PROTOCOL_EID_PING_SENSOR        = 0x17,
    // DYN_PROTOCOL_EID_SET_RUNNING_STATE  = 0x18,
    DYN_PROTOCOL_EID_START_SENSOR       = 0x19,
    DYN_PROTOCOL_EID_STOP_SENSOR        = 0x1A,
    DYN_PROTOCOL_EID_SET_SENSOR_PERIOD  = 0x1B,
    DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT = 0x1C,
    DYN_PROTOCOL_EID_FLUSH_SENSOR       = 0x1D,
    DYN_PROTOCOL_EID_SET_SENSOR_BIAS    = 0x1E,
    DYN_PROTOCOL_EID_GET_SENSOR_BIAS    = 0x1F,
    DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX = 0x20,
    DYN_PROTOCOL_EID_GET_SENSOR_DATA    = 0x21,
    DYN_PROTOCOL_EID_GET_SW_REG         = 0x22,
    DYN_PROTOCOL_EID_SET_SENSOR_CFG     = 0x23,
    DYN_PROTOCOL_EID_GET_SENSOR_CFG     = 0x24,
    DYN_PROTOCOL_EID_WRITE_MEMS_REG     = 0x25,
    DYN_PROTOCOL_EID_READ_MEMS_REG      = 0x26,
    DYN_PROTOCOL_EID_TRANSFER_BUFFER    = 0x27,

    /* Events */
    DYN_PROTOCOL_EID_NEW_SENSOR_DATA    = 0x30,
};


/* Forward declaration */
int icm20948_sensor_setup(void);
//void iddwrapper_protocol_event_cb(enum DynProtocolEtype etype, enum DynProtocolEid eid, const DynProtocolEdata_t * edata, void * cookie);
//void iddwrapper_transport_event_cb(enum DynProTransportEvent e, union DynProTransportEventData data, void * cookie);
void sensor_event(const inv_sensor_event_t * event, void * arg);
int handle_command(enum DynProtocolEid eid);
int icm20948_run_selftest(void);
void inv_icm20948_get_st_bias(struct inv_icm20948 * s, int *gyro_bias, int *accel_bias, int * st_bias, int * unscaled);
void InvEMDFrontEnd_busyWaitUsHook(uint32_t us);
int InvEMDFrontEnd_isHwFlowCtrlSupportedHook(void);
int InvEMDFrontEnd_putcharHook(int c);
void build_sensor_event_data(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
void inv_icm20948_sleep(int ms);
int load_dmp3(void);
void check_rc(int rc, const char * msg_context);

extern inv_icm20948_t icm_device;

