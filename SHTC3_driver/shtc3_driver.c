/**
  ******************************************************************************
  * @file           : shtc3_driver.c
  * @author         : Kamil Kośnik, Kacper Radzikowski
  * @date           : 17.06.2025
  * @brief          : todo: write brief
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "shtc3_driver.h"

/* Private macros ------------------------------------------------------------*/
#define CRC8_POLYNOMIAL	0x31
#define CRC8_INIT 		0xFF
#define CRC8_LEN 		1

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Function that implements the default I2C read transaction
 *
 * @param data : Pointer to the data to be read from addr
 * @param data_len : Length of the data transfer
 * @param intf     : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t shtc3_reg_read(uint8_t *data, uint32_t data_len, void *intf);

/**
 * @brief Function that implements the default I2C write transaction
 *
 * @param data : Data (command) to be written to device
 * @param intf : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t shtc3_reg_write(uint16_t data, void *intf);

/**
 * @brief Function that generates a CRC byte for a given data
 *
 * @param data  :
 * @param count :
 *
 * @return CRC byte
 */
static uint8_t generate_crc(const uint8_t *data, uint16_t count);

/**
 * @brief Function that checks the CRC for the received data
 *
 * @param data     :
 * @param count    :
 * @param checksum :
 *
 * @return False on failure or True on success
 */
static bool check_crc(const uint8_t *data, uint16_t count, uint8_t checksum);

static float calc_temp(uint16_t raw_temp);
static float calc_hum(uint16_t raw_hum);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 */
int shtc3_init(shtc3_t *const me, void *i2c_handle, uint8_t dev_addr)
{
	/* Variable to return error code */
	int ret = 0;
	me->i2c_dev.handle = (I2C_HandleTypeDef *)i2c_handle;
	me->i2c_dev.addr = dev_addr;
	me->id = 0;
	me->state = STHC3_IDLE;

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to get the device ID
 */
int shtc3_get_id(shtc3_t *const me)
{
	me->id = 0;

	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_READ_ID, &me->i2c_dev);

	uint8_t data[3] = {0};
	shtc3_reg_read(data, 3, &me->i2c_dev);

	/* Check data received CRC */
	if (!check_crc(data, 2, data[2])) {
		return -1;
	}

	me->id = data[0] << 8 | data[1];

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to get the temperature (°C) and humidity (%)
 */
int shtc3_get_temp_and_hum(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_wakeup(me);

	shtc3_reg_write(SHTC3_CMD_MEAS_T_RH_CLOCKSTR_NM, &me->i2c_dev);

	HAL_Delay(300);

	uint8_t data[6] = {0};
	shtc3_reg_read(data, 6, &me->i2c_dev);

	/* Check data received CRC */
	if (!check_crc(&data[0], 2, data[2])) {
		return -1;
	}

	if (!check_crc(&data[3], 2, data[5])) {
		return -1;
	}

	me->temp = calc_temp((uint16_t)((data[0] << 8) | (data[1])));
	me->hum = calc_hum((uint16_t)((data[3] << 8) | (data[4])));

	/* Return 0 */
	return ret;
}

 /**
 * @brief Function to get the temperature (°C) and humidity (%) in low
 *        power mode
 */
 int shtc3_get_temp_and_hum_lpm(shtc3_t *const me, float *temp, float *hum)
 {
		/* Variable to return error code */
		int ret = 0;

		shtc3_wakeup(me);

		shtc3_reg_write(SHTC3_CMD_MEAS_T_RH_CLOCKSTR_LPM, &me->i2c_dev);

		HAL_Delay(1);

		uint8_t data[6] = {0};
		shtc3_reg_read(data, 6, &me->i2c_dev);

		/* Check data received CRC */
		if (!check_crc(&data[0], 2, data[2])) {
			return -1;
		}

		if (!check_crc(&data[3], 2, data[5])) {
			return -1;
		}

		me->temp = calc_temp((uint16_t)((data[0] << 8) | (data[1])));
		me->hum = calc_hum((uint16_t)((data[3] << 8) | (data[4])));

		/* Return 0 */
		return ret;
 }

/**
 * @brief Function to get the temperature (°C) and humidity (%). This function
 *        polls every 1 ms until measumente is ready
 */
int shtc3_get_temp_and_hum_polling(shtc3_t *const me, float *temp, float *hum)
{
	/* Variable to return error code */
	int ret = 0;

	/* todo: write */

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to put the device in sleep mode
 */
int shtc3_sleep(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_SLEEP, &me->i2c_dev);

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to wakeup the device from sleep mode
 */
int shtc3_wakeup(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_WAKEUP, &me->i2c_dev);

	HAL_Delay(1);

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to perfrom a software reset of the device
 */
int shtc3_soft_reset(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_SOFT_RESET, &me->i2c_dev);

	/* Return 0 */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 */
static int8_t shtc3_reg_read(uint8_t *data, uint32_t data_len, void *intf)
{
	shtc3_i2c_t *i2c_dev = (shtc3_i2c_t *)intf;

	if (HAL_I2C_Master_Receive(i2c_dev->handle, (i2c_dev->addr << 1) | 0x01,
			data, data_len, 100) > 0) {
		return -1;
	}

	return 0;
}

/**
 * @brief Function that implements the default I2C write transaction
 */
static int8_t shtc3_reg_write(uint16_t data, void *intf)
{
	uint8_t buffer[SHTC3_I2C_BUFFER_LEN_MAX] = {
			(uint8_t)((data >> 8) & 0xFF),
			(uint8_t)(data & 0xFF)
	};

	/* Transmit buffer */
	shtc3_i2c_t *i2c_dev = (shtc3_i2c_t *)intf;

	if (HAL_I2C_Master_Transmit(i2c_dev->handle, i2c_dev->addr << 1, buffer, 2,
			100)) {
		return -1;
	}

	return 0;
}

/**
 * @brief Function that generates a CRC byte for a given data
 */
static uint8_t generate_crc(const uint8_t *data, uint16_t count) {
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for (current_byte = 0; current_byte < count; ++current_byte) {
  	crc ^= (data[current_byte]);

  	for (crc_bit = 8; crc_bit > 0; --crc_bit) {
  		if (crc & 0x80) {
  			crc = (crc << 1) ^ CRC8_POLYNOMIAL;
  		}
  		else {
  			crc = (crc << 1);
  		}
  	}
  }
  return crc;
}

/**
 * @brief Function that checks the CRC for the received data
 */
static bool check_crc(const uint8_t *data, uint16_t count, uint8_t checksum) {
	if (generate_crc(data, count) != checksum) {
		return false;
	}

	return true;
}

static float calc_temp(uint16_t raw_temp)
{
	return 175 * (float)raw_temp / 65536.0f - 45.0f;
}

static float calc_hum(uint16_t raw_hum)
{
	return 100 * (float)raw_hum / 65536.0f;
}
/***************************** END OF FILE ************************************/
