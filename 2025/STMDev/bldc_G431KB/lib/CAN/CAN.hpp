#ifndef __CAN__
#define __CAN__

#include "main.h"
#include "fdcan.h"
#include "stdio.h"

#ifdef __cplusplus

extern "C" {
class CAN {
  public:
    typedef struct {
        unsigned int stdId;
        unsigned char data[8];
    } CANData;

    CAN(FDCAN_HandleTypeDef *_fdcan, uint32_t _myId)
        : hfdcan(_fdcan), myId(_myId) {
    }

    void init() {
        if (HAL_FDCAN_Start(hfdcan) != HAL_OK) Error_Handler();
        FDCAN_FilterTypeDef filter = {
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType = FDCAN_FILTER_MASK,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0,
            .FilterID2 = 0,
        };
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) Error_Handler();                                  // フィルター設定
        if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler(); // 受信割り込み設定
        printf("- CAN init\n");
    }

    void
    send(CANData &canData) {
        txHeader.Identifier = canData.stdId;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, canData.data) != HAL_OK) {
            printf("error: CAN send\n");
        }
        // while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 3)
        //     ;
    }

    void recv(CANData &canData) {
        FDCAN_RxHeaderTypeDef rxHeader;

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0)
            ;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, canData.data) == HAL_OK) {
            canData.stdId = rxHeader.Identifier;
        } else {
            printf("error: CAN recv\n");
        }
    }

    FDCAN_HandleTypeDef *getHcan() {
        return hfdcan;
    }

  private:
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t myId;

    FDCAN_TxHeaderTypeDef txHeader;
    FDCAN_TxHeaderTypeDef rxHeader;
};
};
#endif
#endif