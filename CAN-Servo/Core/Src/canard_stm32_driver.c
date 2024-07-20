/*
 * canard_stm32_driver.c
 *
 *  Created on: Jul 8, 2024
 *      Author: Roni Kant
 */

#include <canard.h>
#include "stm32l4xx_hal.h"
#include <string.h>


/**
  * @brief  Process CAN message from RxLocation FIFO into rx_frame
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxLocation Location of the received message to be read.
  * @param  rx_frame pointer to a CanardCANFrame structure where the received CAN message will be
  *         stored.
  * @retval ret == 1: OK, ret < 0: CANARD_ERROR, ret == 0: Check hcan->ErrorCode
  */
int16_t canardSTM32Receive(CAN_HandleTypeDef *hcan, uint32_t RxLocation, CanardCANFrame *const rx_frame) {
    if (rx_frame == NULL) {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, RxLocation, &RxHeader, RxData) == HAL_OK) {
        // Process ID to canard format
        rx_frame->id = RxHeader.StdId;

        if (RxHeader.IDE == CAN_ID_EXT) { // canard will only process the message if it is extended ID
            rx_frame->id = RxHeader.ExtId;
            rx_frame->id |= CANARD_CAN_FRAME_EFF;
        }

        if (RxHeader.RTR == CAN_RTR_REMOTE) { // canard won't process the message if it is a remote frame
            rx_frame->id |= CANARD_CAN_FRAME_RTR;
        }

        rx_frame->data_len = RxHeader.DLC;
        memcpy(rx_frame->data, RxData, RxHeader.DLC);

        // assume a single interface
        rx_frame->iface_id = 0;

        return 1;
    }

    // Either no CAN msg to be read, or an error that can be read from hcan->ErrorCode
    return 0;
}

/**
  * @brief  Process tx_frame CAN message into Tx FIFO/Queue and transmit it
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  tx_frame pointer to a CanardCANFrame structure that contains the CAN message to
  *         transmit.
  * @retval ret == 1: OK, ret < 0: CANARD_ERROR, ret == 0: Check hcan->ErrorCode
  */
int16_t canardSTM32Transmit(CAN_HandleTypeDef *hcan, const CanardCANFrame* const tx_frame) {
    if (tx_frame == NULL) {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if (tx_frame->id & CANARD_CAN_FRAME_ERR) {
        return -CANARD_ERROR_INVALID_ARGUMENT; // unsupported frame format
    }

    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    // Process canard id to STM CAN header format
    if (tx_frame->id & CANARD_CAN_FRAME_EFF) {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = tx_frame->id & CANARD_CAN_EXT_ID_MASK;
    } else {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = tx_frame->id & CANARD_CAN_STD_ID_MASK;
    }

    TxHeader.DLC = tx_frame->data_len;

    if (tx_frame->id & CANARD_CAN_FRAME_RTR) {
        TxHeader.RTR = CAN_RTR_REMOTE;
    } else {
        TxHeader.RTR = CAN_RTR_DATA;
    }

    memcpy(TxData, tx_frame->data, TxHeader.DLC);

    uint32_t txMailbox;
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &txMailbox) == HAL_OK) {
        return 1;
    }

    // This might be for many reasons including the Tx Mailbox being full, the error can be read from hcan->ErrorCode
    return 0;
}
