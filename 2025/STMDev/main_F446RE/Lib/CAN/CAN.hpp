#ifndef __CANBus__
#define __CANBus__

#include "main.h"
#include "can.h"
#include "stdio.h"

#ifdef __cplusplus

extern "C" {
class CANBus {
  public:
    typedef struct {
        unsigned int stdId;
        unsigned char data[8];
    } CANData;

    CANBus(CAN_HandleTypeDef *_hcan, uint32_t _myId)
        : hcan(_hcan), myId(_myId) {
    }

    void init() {
        // バスオフ(エラー多発でCANが自ら通信を止める状態)から自動で復帰させる。
        // CubeMXの設定(AutoBusOff=DISABLE)のままだと、一度バスオフになるとリセットするまでCANが使えない。
        // 設定は初期化モード中(HAL_CAN_Start前)に行う
        SET_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
        if (HAL_CAN_Start(hcan) != HAL_OK) Error_Handler();
        CAN_FilterTypeDef filter = {
            .FilterIdHigh = 0,                        // フィルターID(上位16ビット)
            .FilterIdLow = 0,                         // フィルターID(下位16ビット)
            .FilterMaskIdHigh = 0,                    // フィルターマスク(上位16ビット)
            .FilterMaskIdLow = 0,                     // フィルターマスク(下位16ビット)
            .FilterFIFOAssignment = CAN_FILTER_FIFO0, // フィルターに割り当てるFIFO
            .FilterBank = 0,                          // フィルターバンクNo
            .FilterMode = CAN_FILTERMODE_IDMASK,      // フィルターモード
            .FilterScale = CAN_FILTERSCALE_32BIT,     // フィルタースケール
            .FilterActivation = ENABLE,               // フィルター無効／有効
            .SlaveStartFilterBank = 14,               // スレーブCANの開始フィルターバンクNo
        };
        if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) Error_Handler();                             // フィルター設定
        if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler(); // 受信割り込み設定
        // printf("- CAN init\n");
    }

    void send(CANData &canData) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
            uint32_t TxMailbox;
            txHeader.StdId = canData.stdId;
            txHeader.RTR = CAN_RTR_DATA;
            txHeader.IDE = CAN_ID_STD;
            txHeader.DLC = 8;
            txHeader.TransmitGlobalTime = DISABLE;
            HAL_CAN_AddTxMessage(hcan, &txHeader, canData.data, &TxMailbox);
            // printf("Mailbox:%ld\n", TxMailbox);
        } else {
            txDropCount++;  // 送信メールボックスが全て埋まっていて送れなかった
        }
        // printf("send\n");
    }

    void recv(CANData &canData) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canData.data) == HAL_OK) {
            canData.stdId = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
        } else {
            printf("error: CAN recv\n");
        }
    }

    CAN_HandleTypeDef *getHcan() {
        return hcan;
    }

    uint32_t txDropCount = 0;  // 異常カウンタ: 送れずに捨てたフレーム数

  private:
    CAN_HandleTypeDef *hcan;
    uint32_t myId;

    CAN_TxHeaderTypeDef txHeader;
    CAN_RxHeaderTypeDef rxHeader;
};
};
#endif
#endif