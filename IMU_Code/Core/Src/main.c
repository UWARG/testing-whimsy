/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "ucpd.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#define CS_PIN GPIO_PIN_4
#define CS_GPIO_PORT GPIOA
#define UB0_REG_WHO_AM_I 0x75
#define REG_BANK_SEL  0x76
#define UB0_REG_DEVICE_CONFIG  0x11
#define UB0_REG_PWR_MGMT0 0x4E
#define UB0_REG_TEMP_DATA1 0x1D

int begin();
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
void writeRegister(uint8_t subAddress, uint8_t data);
int setBank(uint8_t bank);
void setLowNoiseMode();
void reset();
uint8_t whoAmI();
void AGT(uint8_t *dataBuffer);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

	uint8_t tx = subAddress | 0x80;
	uint8_t dummy_tx[count];
	memset(dummy_tx, 0, count*sizeof(dummy_tx[0]));
	uint8_t dummy_rx;
	HAL_StatusTypeDef ret;

	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET);

	ret = HAL_SPI_TransmitReceive(&hspi1, &tx, &dummy_rx, 1, HAL_MAX_DELAY);

	ret = HAL_SPI_TransmitReceive(&hspi1, dummy_tx, dest, count, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);

	return 1;
}

void writeRegister(uint8_t subAddress, uint8_t data){
	uint8_t tx_buf[2] = {subAddress, data};
	HAL_StatusTypeDef ret;

	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(&hspi1, tx_buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);

//	readRegisters(subAddress, 1, &dummy_rx);
//
//	if(dummy_rx == data) {
//		return 1;
//	  }
//	  else{
//		return -1;
//	  }
//
//	return 1;
}

int begin(){
	HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET);
	reset();
	uint8_t address = whoAmI();
	setLowNoiseMode();
	return address;
}

int setBank(uint8_t bank){
	writeRegister(REG_BANK_SEL , 0);
	return 1;
}

void setLowNoiseMode(){
	writeRegister(UB0_REG_PWR_MGMT0, 0x0F);
}

void reset(){
	setBank(0);

	writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

	HAL_Delay(1);
}

uint8_t whoAmI(){
	uint8_t buffer;
	readRegisters(UB0_REG_WHO_AM_I, 1, &buffer);
	return buffer;
}

void AGT(uint8_t *dataBuffer){
	readRegisters(UB0_REG_TEMP_DATA1, 14, dataBuffer);
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
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
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_UCPD1_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  uint8_t address = begin();
  printf("Address: %u\r\n", address);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t myBuffer[14];
	  int16_t rawMeas[7];
	  int16_t accel[3];
	  int16_t gyro[3];
	  AGT(myBuffer);
	  for (size_t i=0; i<7; i++) {
	      rawMeas[i] = ((int16_t)myBuffer[i*2] << 8) | myBuffer[i*2+1];
	   }
	  int16_t temperature = rawMeas[0];
	  for (size_t i=0; i<3; i++) {
		  accel[i] = rawMeas[i+1];
	  }

	  //when holding the icm42688 flat on the table with it oriented
	  //so that you can read the words and assuming a positive value,
	  //the following are true:
	  //accel[0] is to the right
	  //accel[1] is forwards
	  //accel[2] is downwards


	  for (size_t i=0; i<3; i++) {
		  gyro[i] = rawMeas[i+4];
	  }

	  //same orientation
	  //gyro[0] rotating forward-down
	  //gyro[1] rotating left-down
	  //gyro[2] is rotating counter clock wise

    /* USER CODE END WHILE */

	  HAL_Delay(10);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
