/*
 * led_driver.cpp
 *
 *  Created on: Jul 15, 2024
 *      Author: ronik
 */

#include "led_driver.hpp"
#include "stm32l5xx_hal.h"

void LEDDriver::flashLED() {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}

void LEDDriver::wait(uint32_t timeout_ms) {
	HAL_Delay(timeout_ms);
}


