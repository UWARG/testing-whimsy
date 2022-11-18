/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t AIRSPEED_ADDR = 0xF42AE << 1; // Use 8-bit address
static const uint8_t REG_TEMP = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  HAL_StatusTypeDef ret;
  uint8_t buf[12];
  int16_t val;
  float temp_c;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Tell AIRSPEED that we want to read from the airspeed sensor
    buf[0] = REG_TEMP;
    ret = HAL_I2C_Master_Transmit(&hi2c1, AIRSPEED_ADDR, buf, 1, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
      strcpy((char*)buf, "Error Tx\r\n");
    } else {

      // Read 2 bytes from the temperature register
      ret = HAL_I2C_Master_Receive(&hi2c1, AIRSPEED_ADDR, buf, 2, HAL_MAX_DELAY);
      if ( ret != HAL_OK ) {
        strcpy((char*)buf, "Error Rx\r\n");
      } else {

        //Combine the bytes
        val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);

        // Convert to 2's complement, since temperature can be negative
        if ( val > 0x7FF ) {
          val |= 0xF000;
        }

        // Convert to float temperature value (Celsius)
        temp_c = val * 0.0625;

        // Convert temperature to decimal format
        temp_c *= 100;
        sprintf((char*)buf,
              "%u.%u C\r\n",
              ((unsigned int)temp_c / 100),
              ((unsigned int)temp_c % 100));
      }
    }

    // Send out buffer (temperature or error message)
    HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

    // Wait
    HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// Rest of auto-generated Cube functions
// ...
