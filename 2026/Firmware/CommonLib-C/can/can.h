#ifndef CAN_H_
#define CAN_H_

#include "main.h"

typedef struct {
      uint32_t stdId;
      uint8_t data[8];
} CanData;

typedef struct {
      CAN_HandleTypeDef* hcan;
      uint32_t myId;

      CAN_TxHeaderTypeDef txHeader;
      CAN_RxHeaderTypeDef rxHeader;
} Can;

static inline void Can_Init(Can* self, CAN_HandleTypeDef* hcan, uint32_t myId) {
      self->hcan = hcan;
      self->myId = myId;

      if (HAL_CAN_Start(self->hcan) != HAL_OK) Error_Handler();

      CAN_FilterTypeDef filter = {
          .FilterIdHigh = 0,                         // フィルターID(上位16ビット)
          .FilterIdLow = 0,                          // フィルターID(下位16ビット)
          .FilterMaskIdHigh = 0,                     // フィルターマスク(上位16ビット)
          .FilterMaskIdLow = 0,                      // フィルターマスク(下位16ビット)
          .FilterFIFOAssignment = CAN_FILTER_FIFO0,  // フィルターに割り当てるFIFO
          .FilterBank = 0,                           // フィルターバンクNo
          .FilterMode = CAN_FILTERMODE_IDMASK,       // フィルターモード
          .FilterScale = CAN_FILTERSCALE_32BIT,      // フィルタースケール
          .FilterActivation = ENABLE,                // フィルター無効／有効
          .SlaveStartFilterBank = 14,                // スレーブCANの開始フィルターバンクNo
      };

      if (HAL_CAN_ConfigFilter(self->hcan, &filter) != HAL_OK) Error_Handler();                              // フィルター設定
      if (HAL_CAN_ActivateNotification(self->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();  // 受信割り込み設定
}

static inline void Can_Send(Can* self, const CanData* canData) {
      if (HAL_CAN_GetTxMailboxesFreeLevel(self->hcan) > 0) {
            uint32_t TxMailbox;
            self->txHeader.StdId = canData->stdId;
            self->txHeader.RTR = CAN_RTR_DATA;
            self->txHeader.IDE = CAN_ID_STD;
            self->txHeader.DLC = 8;
            self->txHeader.TransmitGlobalTime = DISABLE;
            HAL_CAN_AddTxMessage(self->hcan, &self->txHeader, (uint8_t*)canData->data, &TxMailbox);
      }
}

static inline void Can_Recv(Can* self, CanData* canData) {
      if (HAL_CAN_GetRxMessage(self->hcan, CAN_RX_FIFO0, &self->rxHeader, canData->data) == HAL_OK) {
            canData->stdId = (self->rxHeader.IDE == CAN_ID_STD) ? self->rxHeader.StdId : self->rxHeader.ExtId;
      }
}

static inline CAN_HandleTypeDef* Can_GetHandle(Can* self) {
      return self->hcan;
}

#endif