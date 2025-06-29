/**
  ******************************************************************************
  * @file           : shtc3_driver.h
  * @author         : Kamil Kośnik, Kacper Radzikowski
  * @date           : 17.06.2025
  * @brief          : todo: write brief
  ******************************************************************************
*/

#ifndef SHTC3_DRIVER_H_
#define SHTC3_DRIVER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "i2c.h"

/* Exported Macros -----------------------------------------------------------*/
#define SHTC3_I2C_ADDR						0x70
#define SHTC3_I2C_BUFFER_LEN_MAX			8

#define SHTC3_CMD_READ_ID					0xEFC8 /* command: read ID register */
#define SHTC3_CMD_SOFT_RESET				0x805D /* soft reset */
#define SHTC3_CMD_SLEEP						0xB098 /* sleep */
#define SHTC3_CMD_WAKEUP					0x3517 /* wakeup */

#define SHTC3_CMD_MEAS_T_RH_POLLING_NM		0x7866 /* meas. read T first, clock stretching disabled in normal mode */
#define SHTC3_CMD_MEAS_T_RH_CLOCKSTR_NM		0x7CA2 /* meas. read T first, clock stretching enabled in normal mode */
#define SHTC3_CMD_MEAS_RH_T_POLLING_NM		0x58E0 /* meas. read RH first, clock stretching disabled in normal mode */
#define SHTC3_CMD_MEAS_RH_T_CLOCKSTR_NM		0x5C24 /* meas. read RH first, clock stretching enabled in normal mode */
#define SHTC3_CMD_MEAS_T_RH_POLLING_LPM		0x609C /* meas. read T first, clock stretching disabled in low power mode */
#define SHTC3_CMD_MEAS_T_RH_CLOCKSTR_LPM	0x6458 /* meas. read T first, clock stretching enabled in low power mode */
#define SHTC3_CMD_MEAS_RH_T_POLLING_LPM		0x401A /* meas. read RH first, clock stretching disabled in low power mode */
#define SHTC3_CMD_MEAS_RH_T_CLOCKSTR_LPM	0x44DE /* meas. read RH first, clock stretching enabled in low power mode */

#define SHTC3_CMD_READ_DATA					"SHTC3 READ"
#define SHTC3_CMD_READ_STATE				"SHTC3 STATE"
#define SHTC3_CMD_SET_PERIOD				"SHTC3 PERIOD:"
#define SHTC3_CMD_SET_SINGLE				"SHTC3 SINGLE"

/* Exported typedef ----------------------------------------------------------*/
typedef struct {
	uint8_t addr;
	I2C_HandleTypeDef *handle;
} shtc3_i2c_t;

typedef enum {
	STHC3_IDLE,
	SHTC3_SINGLE_MEASURE,
	SHTC3_CYCLIC_MEASURE,

	SHTC3_SINGLE_MEASURE_START,
}shtc3_state;

typedef struct {
	shtc3_i2c_t i2c_dev;
	uint16_t id;
	shtc3_state state;
	float temp;
	float hum;
} shtc3_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 *
 * @param me         : Pointer to a shtc3_t instance
 * @param i2c_handle : Pointer to a structure with the data to initialize the
 * 					   I2C device
 * @param dev_addr   : I2C device address
 *
 * @return ESP_OK on success
 */
int shtc3_init(shtc3_t *const me, void *i2c_handle, uint8_t dev_addr);

/**
 * @brief Function to get the device ID
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_get_id(shtc3_t *const me);

/**
 * @brief Function to get the temperature (°C) and humidity (%)
 *
 * @param me   : Pointer to a shtc3_t instance
 * @param temp : Pointer to floating point value, where the calculated
 *               temperature value will be stored
 * @param hum  : Pointer to floating point value, where the calculated
 *               humidity value will be stored
 *
 * @return ESP_OK on success
 */
int shtc3_get_temp_and_hum(shtc3_t *const me);

 /**
 * @brief Function to get the temperature (°C) and humidity (%) in low
 *        power mode
 *
 * @param me   : Pointer to a shtc3_t instance
 * @param temp : Pointer to floating point value, where the calculated
 *               temperature value will be stored
 * @param hum  : Pointer to floating point value, where the calculated
 *               humidity value will be stored
 *
 * @return ESP_OK on success
 */
int shtc3_get_temp_and_hum_lpm(shtc3_t *const me, float *temp, float *hum);

/**
 * @brief Function to get the temperature (°C) and humidity (%). This function
 *        polls every 1 ms until measumente is ready
 *
 * @param me   : Pointer to a shtc3_t instance
 * @param temp : Pointer to floating point value, where the calculated
 *               temperature value will be stored
 * @param hum  : Pointer to floating point value, where the calculated
 *               humidity value will be stored
 *
 * @return ESP_OK on success
 */
int shtc3_get_temp_and_hum_polling(shtc3_t *const me);
int shtc3_raw_read_temp_and_hum(shtc3_t *const me);
int shtc3_raw_write_temp_and_hum(shtc3_t *const me);
/**
 * @brief Function to put the device in sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_sleep(shtc3_t *const me);

/**
 * @brief Function to wakeup the device from sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_wakeup(shtc3_t *const me);

/**
 * @brief Function to perfrom a software reset of the device
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_soft_reset(shtc3_t *const me);

#endif /* SHTC3_DRIVER_H_ */

/***************************** END OF FILE ************************************/
