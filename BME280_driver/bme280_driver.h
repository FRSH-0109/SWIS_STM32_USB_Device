
#ifndef BME280_CUSTOM_DRIVER_H_
#define BME280_CUSTOM_DRIVER_H_

#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define BME280_CMD_READ_DATA				"BME280 READ"
#define BME280_CMD_READ_STATE				"BME280 STATE"
#define BME280_CMD_SET_PERIOD				"BME280 PERIOD:"
#define BME280_CMD_GET_PERIOD				"BME280 PERIOD?"
#define BME280_CMD_SET_SINGLE				"BME280 SINGLE"

#define BME280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BME280_REG_TEMP_LSB    0xFB
#define BME280_REG_TEMP_MSB    0xFA
#define BME280_REG_TEMP        (BME280_REG_TEMP_MSB)
#define BME280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BME280_REG_PRESS_LSB   0xF8
#define BME280_REG_PRESS_MSB   0xF7
#define BME280_REG_PRESSURE    (BME280_REG_PRESS_MSB)
#define BME280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BME280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BME280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BME280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BME280_REG_RESET       0xE0
#define BME280_REG_ID          0xD0
#define BME280_REG_CALIB       0x88
#define BME280_REG_HUM_CALIB   0x88
#define BME280_RESET_VALUE     0xB6
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5

typedef enum {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2 = 1,
    BME280_FILTER_4 = 2,
    BME280_FILTER_8 = 3,
    BME280_FILTER_16 = 4
} BME280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BME280_SKIPPED = 0,          /* no measurement  */
    BME280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BME280_LOW_POWER = 2,        /* oversampling x2 */
    BME280_STANDARD = 3,         /* oversampling x4 */
    BME280_HIGH_RES = 4,         /* oversampling x8 */
    BME280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BME280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BME280_STANDBY_05 = 0,      /* stand by time 0.5ms */
    BME280_STANDBY_62 = 1,      /* stand by time 62.5ms */
    BME280_STANDBY_125 = 2,     /* stand by time 125ms */
    BME280_STANDBY_250 = 3,     /* stand by time 250ms */
    BME280_STANDBY_500 = 4,     /* stand by time 500ms */
    BME280_STANDBY_1000 = 5,    /* stand by time 1s */
    BME280_STANDBY_2000 = 6,    /* stand by time 2s BME280, 10ms BME280 */
    BME280_STANDBY_4000 = 7,    /* stand by time 4s BME280, 20ms BME280 */
} BME280_StandbyTime;

typedef struct {
	uint8_t addr;
	I2C_HandleTypeDef *handle;
} bme280_i2c_t;

typedef enum {
    BME280_MODE_SLEEP = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3
} BME280_Mode;


typedef enum {
	BME280_IDLE,
	BME280_SINGLE_MEASURE,
	BME280_CYCLIC_MEASURE,
	BME280_SINGLE_MEASURE_START,
	BME280_CYCLIC_MEASURE_START,
	BME280_CYCLIC_MEASURE_WAIT,
}bme280_state;

typedef struct {
	bme280_i2c_t i2c_dev;
	uint8_t id;
	bme280_state state;
	BME280_Mode mode;

	uint32_t period_ms;
	uint32_t cyclic_timestamp;

	float raw_press;
	float raw_temp;
	float raw_hum;

	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	/* Humidity compensation for BME280 */
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} bme280_t;

void bme280_init(bme280_t *me, I2C_HandleTypeDef *i2c_handle);
HAL_StatusTypeDef bme280_check_id(bme280_t *me);
HAL_StatusTypeDef bme280_reset(bme280_t *me);
HAL_StatusTypeDef bme280_configure(bme280_t *me);
HAL_StatusTypeDef bme280_read_raw_data(bme280_t *me);
//HAL_StatusTypeDef bme280_start_forced_meas(bme280_t *me);
//HAL_StatusTypeDef bme280_start_normal_meas(bme280_t *me);
HAL_StatusTypeDef bme280_read_config(bme280_t *me);


bool bme280_read_calibration_data(bme280_t *dev);
bool bme280_read_float(bme280_t *me);
bool bme280_read_fixed(bme280_t *me, int32_t *temperature, uint32_t *pressure, uint32_t *humidity);
bool bme280_force_measurement(bme280_t *me);
bool bme280_sleep(bme280_t *me);
bool bme280_is_measuring(bme280_t *me);
/*********************************************************************************************/

void bme280_set_cs(GPIO_PinState state);
HAL_StatusTypeDef bme280_raw_read(bme280_t *me, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
HAL_StatusTypeDef bme280_raw_write(bme280_t *me, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);

#endif /* BME280_CUSTOM_DRIVER_H_ */
