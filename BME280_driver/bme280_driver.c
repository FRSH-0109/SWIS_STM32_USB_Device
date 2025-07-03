
#include "bme280_driver.h"
#include "string.h"

#define BME280_I2C_ADDRESS_0  0x76
#define BME280_I2C_ADDRESS_1  0x77

void bme280_init(bme280_t *me, I2C_HandleTypeDef *i2c_handle)
{
	me->i2c_dev.handle = i2c_handle;
	me->mode = BME280_MODE_NORMAL;
	me->i2c_dev.addr = BME280_I2C_ADDRESS_0;
}

HAL_StatusTypeDef bme280_check_id(bme280_t *me)
{
   return bme280_raw_read(me, BME280_REG_ID, &me->id, 1);
}

HAL_StatusTypeDef bme280_reset(bme280_t *me)
{
	uint8_t val = BME280_RESET_VALUE;
    return bme280_raw_write(me, BME280_REG_RESET, &val, 1);
}

HAL_StatusTypeDef bme280_configure(bme280_t *me)
{
    HAL_StatusTypeDef status;

    // Set oversampling x1 for humidity
    uint8_t osrs_h = BME280_STANDARD;
    status = bme280_raw_write(me, BME280_REG_CTRL_HUM, &osrs_h, 1);
    if (status != HAL_OK) return status;

    // Set oversampling x1 for temp and pressure, forced mode
    uint8_t osrs_t_p_mode = (BME280_STANDARD << 5) | (BME280_STANDARD << 2) | 0x01;
    status = bme280_raw_write(me, BME280_REG_CTRL_MEAS, &osrs_t_p_mode, 1);
    if (status != HAL_OK) return status;

    // Set IIR filter off, standby 0.5ms
    uint8_t config = (0x00 << 5) | (0x00 << 2);
    return bme280_raw_write(me, BME280_REG_CONFIG, &config, 1);
}

HAL_StatusTypeDef bme280_read_raw_data(bme280_t *me)
{
    uint8_t data[8];
    HAL_StatusTypeDef status = bme280_raw_read(me, BME280_REG_PRESS_MSB, data, 6);
    if (status != HAL_OK) return status;

    me->raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    me->raw_temp  = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    me->raw_hum   = (data[6] << 8) | data[7];

    return HAL_OK;
}

bool bme280_is_measuring(bme280_t *me)
{
	uint8_t status;
	if (bme280_raw_read(me, BME280_REG_STATUS, &status, 1) != HAL_OK)
	{
		return false;
	}

	if (status & (1 << 3))
	{
		return true;
	}
	return false;
}

bool bme280_force_measurement(bme280_t *me)
{
	uint8_t ctrl;
	if (bme280_raw_read(me, BME280_REG_CTRL, &ctrl, 1) != HAL_OK)
	{
		return false;
	}
	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= BME280_MODE_FORCED;
	if (bme280_raw_write(me, BME280_REG_CTRL, &ctrl, 1)) {
		return false;
	}
	return true;
}

bool bme280_sleep(bme280_t *me)
{
	uint8_t ctrl;
	if (bme280_raw_read(me, BME280_REG_CTRL, &ctrl, 1) != HAL_OK)
	{
		return false;
	}
	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= BME280_MODE_SLEEP;
	if (bme280_raw_write(me, BME280_REG_CTRL, &ctrl, 1)) {
		return false;
	}
	return true;
}

//HAL_StatusTypeDef bme280_start_forced_meas(bme280_t *me)
//{
//    // Set mode = forced (01) while preserving oversampling bits
//    uint8_t ctrl_meas;
//    HAL_StatusTypeDef status = bme280_raw_read(me, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
//    if (status != HAL_OK) return status;
//
//    ctrl_meas = (ctrl_meas & 0xFC) | 0x01; // set mode[1:0] = 01
//    return bme280_raw_write(me, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
//}
//
//HAL_StatusTypeDef bme280_start_normal_meas(bme280_t *me)
//{
//    // Set mode = forced (01) while preserving oversampling bits
//    uint8_t ctrl_meas;
//    HAL_StatusTypeDef status = bme280_raw_read(me, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
//    if (status != HAL_OK) return status;
//
//    ctrl_meas = (ctrl_meas & 0xFC) | 0x03; // set mode[1:0] = 01
//    return bme280_raw_write(me, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
//}

HAL_StatusTypeDef bme280_read_config(bme280_t *me)
{
    uint8_t ctrl_hum = 0, ctrl_meas = 0, config = 0;
    HAL_StatusTypeDef status;

    status = bme280_raw_read(me, BME280_REG_CTRL_HUM, &ctrl_hum, 1);
    if (status != HAL_OK) return status;

    status = bme280_raw_read(me, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (status != HAL_OK) return status;

    status = bme280_raw_read(me, BME280_REG_CONFIG, &config, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

bool bme280_read_calibration_data(bme280_t *dev)
{
	if ((bme280_raw_read(dev, 0x88,  (uint8_t *)&dev->dig_T1, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x8a, (uint8_t *) &dev->dig_T2, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x8c, (uint8_t *) &dev->dig_T3, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x8e,  (uint8_t *)&dev->dig_P1, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x90, (uint8_t *) &dev->dig_P2, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x92, (uint8_t *) &dev->dig_P3, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x94, (uint8_t *) &dev->dig_P4, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x96, (uint8_t *) &dev->dig_P5, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x98, (uint8_t *) &dev->dig_P6, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x9a, (uint8_t *) &dev->dig_P7, 2) == HAL_OK)
			&&( bme280_raw_read(dev, 0x9c, (uint8_t *) &dev->dig_P8, 2) == HAL_OK)
			&& (bme280_raw_read(dev, 0x9e, (uint8_t *) &dev->dig_P9,2)) == HAL_OK) {

		if ((bme280_raw_read(dev, 0xa1, &dev->dig_H1, 1) == HAL_OK)
					&& (bme280_raw_read(dev, 0xe1, (uint8_t *) &dev->dig_H2, 2)== HAL_OK)
					&& (bme280_raw_read(dev, 0xe3, (uint8_t *)&dev->dig_H3, 1)== HAL_OK)
					&& (bme280_raw_read(dev, 0xe4, (uint8_t *)&dev->dig_H4, 2)== HAL_OK)
					&& (bme280_raw_read(dev, 0xe5, (uint8_t *)&dev->dig_H5, 2)== HAL_OK)
					&& (bme280_raw_read(dev, 0xe7, (uint8_t *) &dev->dig_H6, 1)== HAL_OK))
		{
			dev->dig_H4 = (dev->dig_H4 & 0x00ff) << 4 | (dev->dig_H4 & 0x0f00) >> 8;
			dev->dig_H5 = dev->dig_H5 >> 4;
		}
		else
		{
			return false;
		}

		return true;
	}

	return false;
}

static inline int32_t compensate_temperature(bme280_t *dev, int32_t adc_temp, int32_t *fine_temp)
{
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
			* (int32_t) dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
			* ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
			* (int32_t) dev->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(bme280_t *dev, int32_t adc_press, int32_t fine_temp)
{
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(bme280_t *dev, int32_t adc_hum, int32_t fine_temp)
{
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) dev->dig_H4 << 20)
			- ((int32_t) dev->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) dev->dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) dev->dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) dev->dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) dev->dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}

bool bme280_read_fixed(bme280_t *me, int32_t *temperature, uint32_t *pressure, uint32_t *humidity)
{
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];

	// Need to read in one sequence to ensure they match.
	bme280_raw_read(me, 0xf7, data, 8);

	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensate_temperature(me, adc_temp, &fine_temp);
	*pressure = compensate_pressure(me, adc_pressure, fine_temp);

	int32_t adc_humidity = data[6] << 8 | data[7];
	*humidity = compensate_humidity(me, adc_humidity, fine_temp);

	return true;
}

bool bme280_read_float(bme280_t *me)
{
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	uint32_t fixed_humidity;
	if (bme280_read_fixed(me, &fixed_temperature, &fixed_pressure, &fixed_humidity))
	{
		me->raw_temp = (float) fixed_temperature / 100;
		me->raw_press = (float) fixed_pressure / 256;
		me->raw_hum = (float) fixed_humidity / 1024;
		return true;
	}

	return false;
}

/*********************************************************************************************/

void bme280_set_cs(GPIO_PinState state)
{
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, state);
}

HAL_StatusTypeDef bme280_raw_read(bme280_t *me, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
	return HAL_I2C_Mem_Read(me->i2c_dev.handle, me->i2c_dev.addr << 1, reg_addr, 1, reg_data, len, 100);
}

HAL_StatusTypeDef bme280_raw_write(bme280_t *me, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
	 return HAL_I2C_Mem_Write(me->i2c_dev.handle, me->i2c_dev.addr << 1, reg_addr, 1, reg_data, len, 100);
}



