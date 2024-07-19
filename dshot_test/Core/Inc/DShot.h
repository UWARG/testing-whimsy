/*
 * DShot.h
 *
 *  Created on: Jul 7, 2024
 *      Author: Stanley
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "stm32l5xx_hal.h"

#define DSHOT_DATA_FRAME_LEN	16
#define DSHOT_DMA_BUFFER_LEN	18
#define DSHOT_MAX_THROTTLE		2000
#define DSHOT_RESERVED_VALUES	47
#define DSHOT_150_BIT_1			550
#define DSHOT_150_BIT_0			(DSHOT_150_BIT_1 / 2)

typedef struct DShotConfig {
	TIM_HandleTypeDef *timer;
	uint16_t timerChannel;
	uint16_t timDMAHandleIndex;
	uint16_t timDMASource;
	uint32_t* dmaBuffer;
} DShotConfig_t;

void dshotInit(DShotConfig_t dshotConfig);
uint16_t dshotGetThrottleBits(float throttlePercentage, uint8_t telemetry);
void dshotWrite(DShotConfig_t dshotConfig, float throttlePercentage, uint8_t telemetry);

// Util methods
void dshotUpdateDMABuffer(uint32_t *buffer, uint16_t frame);

#endif /* INC_DSHOT_H_ */
