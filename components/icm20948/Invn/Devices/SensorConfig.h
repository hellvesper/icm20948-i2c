/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
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

/** @defgroup SensorConfig Sensor Configuration
 *  @brief    General sensor configuration types definitions
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_SENSOR_CONFIG_H_
#define _INV_SENSOR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Invn/EmbUtils/InvBool.h"

/** @brief Sensor type identifier definition
 */
enum inv_sensor_config {
	INV_SENSOR_CONFIG_RESERVED = 0,          /**< Reserved config ID: do not use */
	INV_SENSOR_CONFIG_MOUNTING_MATRIX,       /**< 3x3 mounting matrix */
	INV_SENSOR_CONFIG_GAIN,                  /**< 3x3 gain matrix (to correct for cross-axis defect)*/
	INV_SENSOR_CONFIG_OFFSET,                /**< 3d offset vector  */
	INV_SENSOR_CONFIG_CONTEXT,               /**< arbitrary context buffer */
	INV_SENSOR_CONFIG_FSR,                   /**< Full scale range */
	INV_SENSOR_CONFIG_BW,                    /**< Bandwidth */
	INV_SENSOR_CONFIG_AVG,                   /**< Average */
	INV_SENSOR_CONFIG_RESET,                 /**< Reset the specified service */
	INV_SENSOR_CONFIG_POWER_MODE,            /**< Low Power or Low Noise mode */
	INV_SENSOR_CONFIG_MIN_PERIOD,            /**< Request sensor smallest period (highest frequency) */
	INV_SENSOR_CONFIG_CUSTOM   = 128,        /**< Configuration ID above this value are device specific */
	INV_SENSOR_CONFIG_PRED_GRV = 131,        /**< Predictive GRV algo parameter */
	INV_SENSOR_CONFIG_BT_STATE = 132,        /**< Drives whether bias tracker usage */
	INV_SENSOR_CONFIG_REPORT_EVENT = 133,    /**< Drives sensor data event reporting */
	INV_SENSOR_CONFIG_FNM_OFFSET = 134,      /**< 3x1 FNM offset vector (used in VR/HMD cases) */
	INV_SENSOR_CONFIG_ALGO_SETTINGS = 138,   /**< Configure internal DMP/RTL states */
	INV_SENSOR_CONFIG_MAX      = 255,        /**< Absolute maximum value for sensor config */
};

/** @brief Define mounting matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_MOUNTING_MATRIX config ID)
 *         Mounting matrix value can be set (is supported by device implementation) to convert from
 *         sensor reference to system reference.
 *         Value is expetcted to be a rotation matrix.
 */
typedef struct inv_sensor_config_mounting_mtx {
	float matrix[3*3];
} inv_sensor_config_mounting_mtx_t;

/** @brief Define gain matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_GAIN config ID)
 *         Gain matrix value can be set (is supported by device implementation) to correct for
 *         cross-axis defect.
 */
typedef struct inv_sensor_config_gain {
	float gain[3*3];
} inv_sensor_config_gain_t;

/** @brief Define offset vector value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_OFFSET config ID)
 *         Offset value can be set (is supported by device implementation) to correct for bias defect.
 *         If applied to RAW sensor, value is expected to be in lsb.
 *         If applied to other sensor, value is expected to be in sensor unit (g, uT or dps).
 */
typedef struct inv_sensor_config_offset {
	float offset[3];
} inv_sensor_config_offset_t;

/** @brief Define configuration context value
 *         (associated with INV_SENSOR_CONFIG_CONTEXT config ID)
 *         Context is an arbitrary buffer specific to the sensor and device implemetation
 */
typedef struct inv_sensor_config_context {
	uint8_t context[64];
} inv_sensor_config_context_t;

/** @brief Define full-scale range value for accelero, gyro or mangetometer based sensor
 *         (associated with INV_SENSOR_CONFIG_FSR config ID)
 *         Value is expetcted to be expressed in mg, dps and uT for accelero, gyro or mangetometer
 *         eg: +/-2g = 2000
 *             +/-250 dps = 250
 *             +/-2000 uT = 2000
 */
typedef struct inv_sensor_config_fsr {
	uint32_t fsr;
} inv_sensor_config_fsr_t;

/** @brief Define bandwidth value for accelero, gyro based sensor
 *         Value is expected to be expressed in Hz
 */
typedef struct inv_sensor_config_bw {
	uint32_t bw;
} inv_sensor_config_bw_t;

/** @brief Define averaging value for accelero based sensor
 *         avg is expected to be expressed without unity
 *         rate is expected to be expressed in KHz
 */
typedef struct inv_sensor_config_avg {
	uint16_t avg;
	uint16_t rate;
} inv_sensor_config_avg_t;

/** @brief Define chip power mode
 *         (associated with INV_SENSOR_CONFIG_POWER_MODE config ID)
 *         Value is expetcted to be 0 for low power or 1 for low noise
 */
typedef struct inv_sensor_config_powermode {
	uint8_t lowpower_or_highperformance;
} inv_sensor_config_powermode_t;

/** @brief Define min allowed period for a chip
 *         (associated with INV_SENSOR_CONFIG_MIN_PERIOD config ID)
 *         Value is expected to be read from sensor's capabilities,
 *         never to be set from the outside world.
 */
typedef struct inv_sensor_config_min_period {
	uint32_t min_period;
} inv_sensor_config_min_period_t;

/** @brief Define predictive GRV parameters
 *         (associate with INV_SENSOR_CONFIG_PRED_GRV config ID)
 *         method is expected to be one of [0;2]. Latency would be
 *         expected to be positive to have a prediction in the future,
 *         but nothing will block a negative value from being given.
 */
typedef struct inv_sensor_config_pred_grv {
	int8_t method;
	int32_t latency;
} inv_sensor_config_pred_grv_t;

/** @brief Define Bias Tracker parameters
 *         (associated with INV_SENSOR_CONFIG_BT_STATE config ID)
 *         Value is expected to be 0 (resp. 1) to disable
 *         (resp. enable) BT engine.
 */
typedef struct inv_sensor_config_bt_state_t {
	uint32_t enable;
} inv_sensor_config_bt_state_t;

/** @brief Define data event reporting parameter
 *         (associated with INV_SENSOR_CONFIG_REPORT_EVENT config ID)
 *         Value is expected to be 0 (resp. 1) to disable
 *         (resp. enable) data reporting.
 */
typedef struct inv_sensor_config_report_t {
	uint32_t enable;
} inv_sensor_config_report_t;

/** @brief Associated with INV_SENSOR_CONFIG_FNM_OFFSET config ID */
typedef inv_sensor_config_offset_t inv_sensor_config_fnm_offset_t;

/** @brief Define HW algorithms parameter
 *         (associated with INV_SENSOR_CONFIG_ALGO_SETTINGS config ID)
 *         Values are dependant on firmware implementation and not interpreted by this layer
 *         but they would expected to be bit-field value (non shifted).
 */
typedef struct inv_sensor_config_algo_settings {
	uint32_t size;              /**< How many useful values are being sent */
	uint32_t values[15];        /**< Set of value to be applied to the given algorithm */
} inv_sensor_config_algo_settings_t;

/** @brief Define the configuration for the energy expenditure's algorithm
 *  @param age      age in year; Range is (0;100).
 *  @param gender   gender is 0 for men, 1 for female.
 *  @param height   height in centimeter; Range is (50;250) 
 *  @param weight   weight in kg; Range is (3;300)
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_energy_expenditure {
	int32_t age;
	int32_t gender;
	int32_t height;
	int32_t weight;
	uint32_t enableNotify;
} inv_sensor_config_energy_expenditure_t;

/** @brief Define the configuration for the distance's algorithm
 *  @param user_height height of the user in cm
 *  @param enableNotify enable disable notify
 */
typedef struct inv_sensor_config_distance{
	int32_t user_height;
	uint32_t enableNotify;
}inv_sensor_config_distance_t;

/** @brief Define the configuration for BAC
 *  @param state_stable_duration_threshold       The time threshold for minimal stable state, unit: number of samples.
 *                                               Value is number of seconds multiplied by sampling rate. default is 0.
 * @param state_stable_duration_threshold_biking the time threshold for minimal stable state transitioned to biking state, unit: number of samples
 *                                               value is number of seconds multiplied by sampling rate. default is 0.
 *                                               this biking threshold should be larger than the all state. Otherwise, there is no effe
 *  @param enableNotify                          enable disable notify
 */
typedef struct inv_sensor_config_bac{
	int32_t state_stable_duration_threshold;
	int32_t state_stable_duration_threshold_biking;
	uint32_t enableNotify;
}inv_sensor_config_bac_t;

/** @brief Define the configuration for steps counter
 *  @param sb_threshold          The minimum number of steps that must be detected before the pedometer step count begins incrementing.
 *                               This is used to prevent false starts of the pedometer when the user only takes a small number of steps but stops again quickly.
 *                               Once the threshold is exceeded, the pedometer step count increases by it. The pedometer increments regularly thereafter.
 *                               The default value is 7.
 *  @param sb_timer_threshold    This parameter sets the duration of non-walk to exit the current walk mode.
 *                               When the condition is satisfied, the step buffer is reset to be ready for the next walk detection.
 *                               The default value is 150 (3 seconds x 50 Hz).
 *  @param sb_threshold2         The minimum number of low latency steps that must be detected before the pedometer step count begins incrementing. The default value is 2.
 *  @param peak_threshold        The peak threshold is the absolute value of the minimum accelerometer data that is considered a valid step. The default value is 2109832.
 *  @param enableNotify          enable disable notify
 */
typedef struct inv_sensor_config_stepc{
	int32_t sb_threshold;
	int32_t sb_timer_threshold;
	int32_t sb_threshold2;
	int32_t peak_threshold;
	uint32_t enableNotify;
}inv_sensor_config_stepc_t;

/** @brief Define the configuration for floor climb counter
 *  @param floorHeight   floor height in meters; default value is 3072 (3m).  Acceptable range is 2m to 2000m (values over 2000m may cause overflow in the algorithm).
 */
typedef struct inv_sensor_config_floor_climb_counter{
	int32_t floorHeight;
}inv_sensor_config_floor_climb_counter_t;

/** @brief Define the configuration for sleep analysis
 *  @param floorHeight   The minimal duration of non-restless before declaring sleep phase. Unit is minute. Default value is 20.
 */
typedef struct inv_sensor_config_sleep_analysis{
	int32_t onset_interval;
}inv_sensor_config_sleep_analysis_t;

/** @brief Define the configuration for BAC extended
 *  @param still_seconds_max         This parameter sets the maximum value for seconds counter in still activity, unit seconds, default 20.
 *  @param still_seconds_threshold   This parameter sets the decision threshold for seconds counter in still activity, unit seconds, default 12.
 */
typedef struct inv_sensor_config_bac_extended{
	int16_t still_seconds_max;
	int16_t still_seconds_threshold;
}inv_sensor_config_bac_extended_t;

/** @brief Define the configuration for the shake wrist's algorithm
 *  @param nb_oscillations_min    This parameter sets the minimal number of oscillation to detect a Shake wrist, default is 1.
 *  @param nb_oscillations_max    This parameter sets the maximal number of oscillation to detect a Shake wrist, default is 2.
 *  @param min_period             This parameter sets the minimal duration for half oscillation to detect a Shake wrist, default is 3 - [samples] (56.25Hz), This correspond to a period of 0.05s.
 *  @param max_period             This parameter sets the maximal duration for half oscillation to detect a Shake wrist, default is 34 - [samples] (56.25Hz), This correspond to a period of 0.6s.
 *                                This parameter also defines the minimum response time of the algorithm once the shake is completed.
 */
typedef struct inv_sensor_config_shake_wrist{
	uint8_t nb_oscillations_min;
	uint8_t nb_oscillations_max;
	uint8_t min_period;
	uint8_t max_period;
}inv_sensor_config_shake_wrist_t;

/** @brief Define the configuration for the double tap's algorithm
 *  @param minimum_threshold This parameter sets the minimum threshold to reach in order to start a Tap detection. 
 *                           Default value is 2000, recommended range [500 ; 7500]
 *                           minimum_threshold=500 correspond to the higher sensibility, detecting an double tap will be easier, but it will leads to false detections when the device has shock.
 *                           minimum_threshold=7500 is the lower sensibility the strength needed to detect a tap will be high, but it will be quasi insensitive to false detections due to shocks
 *  @param t_max             This parameter sets the maximum time after a Tap event in [sample]. Suggested value is 100 [samples] at 200[Hz]. Possible range : t_max [30 ; 200]
 *                           t_max=0.2s will require the user to do fast double tap, but it will ensure a low responce time
 *                           t_max=1s will allow the user to do slow double tap, but it will increase responce time (due to the triple tap rejection).
 *  @param theta_min_max     This parameter sets the tilt angle of each axis for the screen, there are 6 elements in this vector: min and max range values for each 3 axes.
 *                           These parameters are Q16 fxp values, vector format is [ xmin, xmax, ymin, ymax, zmin, zmax].
 *                           The default values are [-22415, 22415, -16962, 65536, 0, 65536].
 *  @param unlock_delay      This parameter sets the unlock_delay time duration [sample] the position has to be out of LAS before the algorithm switches off.
 *                           Default value 28, equivalent to 0.5s at 56.25Hz.
 */
typedef struct inv_sensor_config_double_tap{
	int16_t minimum_threshold;
	uint16_t t_max;
	int32_t theta_min_max[6];
	int32_t unlock_delay;
}inv_sensor_config_double_tap_t;


/** @brief Define the configuration for the BSCD virtual sensor
 *  @param Age            age in year; Range is (0;100). Default is 35.
 *  @param Gender         gender is 0 for men, 1 for female. Default is 0
 *  @param Height         height in centimeter; Range is (50;250). Default is 175.
 *  @param Weight         weight in kg; Range is (3;300). Default is 75
 *  @param enableNotify   bitmask to enable/disable notify on a a specific sensor event
 *                        bit 0 (1): enable/disable notify on BAC event
 *                        bit 1 (2): enable/disable notify on step counter event
 *                        bit 2 (4): enable/disable notify on energy expenditure event
 *                        bit 3 (8): enable/disable notify on distance event
 */
typedef struct inv_sensor_config_BSCD
{
	int32_t Age;
	int32_t Gender;
	int32_t Height;
	int32_t Weight;
	uint32_t enableNotify;
} inv_sensor_config_BSCD_t;

/** @brief Define the configuration for PPG PAH8001
 *  @param Touch_Minimum_Threshold PPG Value Minimum Threshold; Default is 1E7.
 */
typedef struct inv_sensor_config_PPG_PAH8001
{
	uint32_t Touch_Minimum_Threshold;
} inv_sensor_config_PPG_PAH8001_t;

/** @brief Define the configuration for PPG PAH8002
 *  @param Touch_Upper_Threshold PPG Value Upper Threshold; Default is 600.
 *  @param Touch_Lower_Threshold PPG Value Lower Threshold; Default is 512.
 */
typedef struct inv_sensor_config_PPG_PAH8002
{
	uint16_t Touch_Upper_Threshold;
	uint16_t Touch_Lower_Threshold;
} inv_sensor_config_PPG_PAH8002_t;

/** @brief Define the configuration for PPG ADPD174
 *  @param Touch_Minimum_Threshold PPG Value Minimum Threshold; Default is 1E4.
 */
typedef struct inv_sensor_config_PPG_ADPD174
{
	uint32_t Touch_Minimum_Threshold;
} inv_sensor_config_PPG_ADPD174_t;

/** @brief Define the configuration for PPG PPS960
 *  @param Touch_Minimum_Threshold PPG Value Minimum Threshold; Default is 50000.
 */
typedef struct inv_sensor_config_PPG_PPS960
{
	uint32_t Touch_Minimum_Threshold;
} inv_sensor_config_PPG_PPS960_t;

/** @brief Define the configuration for Sedentary Reminder
 *  @param alarm_parameter_in_second Maximum Sit Duration Condition before Alarm occurs.
 *  @param hysteresis_parameter_in_second Minimum Duration of non-sit condition to reset Sedentary Time Counter.
 */
typedef struct inv_sensor_config_sedentary_reminder {
	uint32_t alarm_parameter_in_second;
	uint32_t hysteresis_parameter_in_second;
} inv_sensor_config_sedentary_reminder_t;

#ifdef __cplusplus
}
#endif

#endif /* _INV_SENSOR_CONFIG_H_ */

/** @} */
