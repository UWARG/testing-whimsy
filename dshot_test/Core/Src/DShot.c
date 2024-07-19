/*
 * DShot.c
 *
 *  Created on: Jul 7, 2024
 *      Author: Stanley
 */

#include "DShot.h"

void dshotInit(DShotConfig_t dshotConfig) {
	HAL_TIM_PWM_Start(dshotConfig.timer, dshotConfig.timerChannel);

	DMA_HandleTypeDef* dmaHandle = dshotConfig.timer->hdma[dshotConfig.timDMAHandleIndex];
	uint32_t dmaSrcAddr = (uint32_t)(dshotConfig.dmaBuffer);
	uint32_t dmaDestAddr = 0;

	switch (dshotConfig.timerChannel) {
		case TIM_CHANNEL_1:
			dmaDestAddr = (uint32_t)&dshotConfig.timer->Instance->CCR1;
			break;

		case TIM_CHANNEL_2:
			dmaDestAddr = (uint32_t)&dshotConfig.timer->Instance->CCR2;
			break;

		case TIM_CHANNEL_3:
			dmaDestAddr = (uint32_t)&dshotConfig.timer->Instance->CCR3;
			break;

		case TIM_CHANNEL_4:
			dmaDestAddr = (uint32_t)&dshotConfig.timer->Instance->CCR4;
			break;
	}

	// DMA should be started in circular mode
	HAL_DMA_Start(dmaHandle, dmaSrcAddr, dmaDestAddr, DSHOT_DMA_BUFFER_LEN);

	dshotWrite(dshotConfig, 0.0f, 0);
	HAL_Delay(1000);
}

uint16_t dshotGetThrottleBits(float throttlePercentage, uint8_t telemetry) {
	uint16_t frame = 0;

	// Set throttle bits
	frame = (uint16_t)(DSHOT_MAX_THROTTLE * throttlePercentage / 100) + DSHOT_RESERVED_VALUES;

	// Set telemetry bit
	frame = (frame << 1 ) | (telemetry ? 1 : 0);

	// Calculating checksum... splitting first 12 bits into 3 nibbles and XORing
	uint16_t checksum = ((frame ^ (frame >> 4) ^ (frame >> 8))) & 0x000F;

	// Append checkSum to the frame
	frame = (frame << 4) | checksum;

	return frame;
}

void dshotWrite(DShotConfig_t dshotConfig, float throttlePercentage, uint8_t telemetry) {
	// Disable timer DMA to avoid DMA transfers while the DMA buffer is being updated
	__HAL_TIM_DISABLE_DMA(dshotConfig.timer, dshotConfig.timDMASource);

	dshotUpdateDMABuffer(dshotConfig.dmaBuffer, dshotGetThrottleBits(throttlePercentage, telemetry));

	__HAL_TIM_ENABLE_DMA(dshotConfig.timer, dshotConfig.timDMASource);
}

void dshotUpdateDMABuffer(uint32_t *buffer, uint16_t frame) {
	  /* DSHOT data frame (16 bits total):
	   *
	   *           b0 b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11 b12 b13 b14 b15
	   *           |-------------------------------| |-| |-------------|
	   *                      Throttle Data           ^     Checksum
	   *                                              |
	   *                                           Telemetry
	   */

	  // Convert frame bits into PWM duty cycles in DMA buffer
	  for (uint8_t i = 0; i < DSHOT_DATA_FRAME_LEN; ++i) {
		  buffer[i] = (frame & 0x8000 ? DSHOT_150_BIT_1 : DSHOT_150_BIT_0);
		  frame <<= 1;
	  }

	  // Stuff the remainig bits with 0 to generate a low signal indicating frame reset
	  for (uint8_t i = DSHOT_DATA_FRAME_LEN; i < DSHOT_DMA_BUFFER_LEN; ++i) {
		  buffer[i] = 0;
	  }
}
