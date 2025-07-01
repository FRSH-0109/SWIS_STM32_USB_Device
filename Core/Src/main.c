/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "shtc3_driver.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SIMULATE 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
shtc3_t shtc3_sensor;
uint8_t report[64] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void usbSend(uint8_t *report, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  if(0 == SIMULATE)
  {
	  shtc3_init(&shtc3_sensor, &hi2c1, SHTC3_I2C_ADDR);
	  shtc3_wakeup(&shtc3_sensor);
	  shtc3_get_id(&shtc3_sensor);
	  shtc3_sleep(&shtc3_sensor);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(0 == SIMULATE)
	{
		uint8_t ret	= 0;
		switch(shtc3_sensor.state)
		{
			case SHTC3_SINGLE_MEASURE_START:
				 ret = shtc3_raw_write_temp_and_hum(&shtc3_sensor);
				if(0 == ret) {shtc3_sensor.state = SHTC3_SINGLE_MEASURE;}
			case SHTC3_SINGLE_MEASURE:
				ret = shtc3_raw_read_temp_and_hum(&shtc3_sensor);
				if(0 == ret)
				{
					shtc3_sleep(&shtc3_sensor);
					memcpy(&report[0], &shtc3_sensor.temp, sizeof(shtc3_sensor.temp));
					memcpy(&report[4], &shtc3_sensor.hum, sizeof(shtc3_sensor.hum));
					usbSend(report, 8);
					shtc3_sensor.state = SHTC3_IDLE;
				}
				break;
			case SHTC3_CYCLIC_MEASURE_START:
				ret = shtc3_raw_write_temp_and_hum(&shtc3_sensor);
				if(0 == ret)
				{
					shtc3_sensor.state = SHTC3_CYCLIC_MEASURE;
					shtc3_sensor.cyclic_timestamp = HAL_GetTick();
				}
				break;
			case SHTC3_CYCLIC_MEASURE:
				ret = shtc3_raw_read_temp_and_hum(&shtc3_sensor);
				if(0 == ret)
				{
					shtc3_sleep(&shtc3_sensor);
					memcpy(&report[0], &shtc3_sensor.temp, sizeof(shtc3_sensor.temp));
					memcpy(&report[4], &shtc3_sensor.hum, sizeof(shtc3_sensor.hum));
					usbSend(report, 8);
					shtc3_sensor.state = SHTC3_CYCLIC_MEASURE_WAIT;
				}
				break;
			case SHTC3_CYCLIC_MEASURE_WAIT:
				if(HAL_GetTick() - shtc3_sensor.cyclic_timestamp >= shtc3_sensor.period_ms)
				{
					shtc3_sensor.state = SHTC3_CYCLIC_MEASURE_START;
				}
				break;
			case SHTC3_IDLE:
			default:
	//			shtc3_get_temp_and_hum(&shtc3_sensor);
				break;
		}
	}
	else
	{
		shtc3_sensor.state = SHTC3_IDLE;
		shtc3_sensor.temp = (float)rand()/(float)(RAND_MAX/50);
		shtc3_sensor.hum = (float)rand()/(float)(RAND_MAX/100);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static bool compareStrings(uint8_t *buffer, char *str, uint16_t max_len)
{
	return (0 == strncmp((char *)buffer, str, (strlen(str) > max_len ? max_len : strlen(str))));
}

static void usbSend(uint8_t *report, uint8_t len)
{
	if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		while (((USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData)->state != CUSTOM_HID_IDLE);
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
	}
}

static bool shtc3ParsePeriod(const uint8_t *buffer, uint32_t *period_out)
{
    if (!buffer || !period_out)
        return false;

    uint32_t prefix_len = strlen(SHTC3_CMD_SET_PERIOD);

    // Sprawdzenie prefiksu
    if (strncmp((const char *)buffer, SHTC3_CMD_SET_PERIOD, prefix_len) != 0)
        return false;

    // Wskaźnik do miejsca po "SHTC3 PERIOD:"
    const char *number_str = (const char *)(buffer + prefix_len);

    // Parsowanie liczby
    char *endptr;
    unsigned long val = strtoul(number_str, &endptr, 10);

    // Sprawdzenie, czy udało się sparsować oraz czy nie przekroczono uint32_t
    if (number_str == endptr || val > UINT32_MAX || val < 0)
        return false;

    *period_out = (uint32_t)val;
    return true;
}

void usb_parser(uint8_t *buffer, uint16_t max_len)
{
	memset(report, 0,sizeof(report));
	if(compareStrings(buffer, SHTC3_CMD_READ_DATA, max_len))
	{
		sprintf((char *)report, "SHTC3 DATA: %.3f %.3f", shtc3_sensor.temp, shtc3_sensor.hum);
		usbSend(report, strlen((char *)report));
	}
	else if(compareStrings(buffer, SHTC3_CMD_READ_STATE, max_len))
	{
		switch (shtc3_sensor.state)
		{
			case SHTC3_IDLE:
				sprintf((char *)report, "SHTC3 IDLE");
				break;
			case SHTC3_SINGLE_MEASURE_START:
			case SHTC3_SINGLE_MEASURE:
				sprintf((char *)report, "SHTC3 SINGLE MEASURE");
				break;
			case SHTC3_CYCLIC_MEASURE:
				sprintf((char *)report, "SHTC3 CYCLIC MEASURE");
				break;
			default:
				sprintf((char *)report, "SHTC3 UNKNOW");
				break;
		}
		usbSend(report, strlen((char *)report));
	}
	else if(compareStrings(buffer, SHTC3_CMD_SET_SINGLE, max_len))
	{
		switch (shtc3_sensor.state)
		{
			case SHTC3_IDLE:
				shtc3_sensor.state = SHTC3_SINGLE_MEASURE_START;
				break;
			default:
				sprintf((char *)report, "SHTC3 BUSY");
				break;
		}
	}
	else if(compareStrings(buffer, SHTC3_CMD_SET_PERIOD, max_len))
	{
		if(strlen((char *)buffer) < strlen(SHTC3_CMD_SET_PERIOD))
		{
			sprintf((char *)report, "SHTC3 PERIOD INCORRECT");
			usbSend(report, strlen((char *)report));
		}
		uint32_t periodTmp = 0;
		bool ret = shtc3ParsePeriod(buffer, &periodTmp);
		if(ret)
		{
			shtc3_sensor.period_ms = periodTmp;
			if(periodTmp != 0) { shtc3_sensor.state = SHTC3_CYCLIC_MEASURE_START;}
			else {shtc3_sensor.state = SHTC3_IDLE;}
		}
		else
		{
			sprintf((char *)report, "SHTC3 PERIOD INCORRECT");
			usbSend(report, strlen((char *)report));
		}
	}
	else if(compareStrings(buffer, SHTC3_CMD_GET_PERIOD, max_len))
	{
		sprintf((char *)report, "SHTC3 PERIOD: %ld", shtc3_sensor.period_ms);
		usbSend(report, strlen((char *)report));
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
