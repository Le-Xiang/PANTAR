#include "bsp_can.h"
#include "can.h"

/* ========= These are global variable definitions (not extern) ========= */
moto_measure_t moto_chassis[4] = {0};
moto_measure_t moto_info = {0};
/* ===================================================================== */

/* Filter init: receive all, FIFO0
   Note: do NOT start CAN/enable IRQ here; let main.c do it (Plan A) */
void my_can_filter_init_recv_all(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef f = {0};

    if (hcan->Instance == CAN1) {
        f.FilterBank = 0;
        f.SlaveStartFilterBank = 14;  // banks >=14 for CAN2
    } else {
        f.FilterBank = 14;
        f.SlaveStartFilterBank = 14;
    }

    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;

    (void)HAL_CAN_ConfigFilter(hcan, &f);

    /* Plan A: start & enable interrupts in main.c to avoid double-start
       HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
       HAL_CAN_Start(hcan);
    */
}

/* RX callback: parse four motor feedback frames, including total_angle accumulation */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) return;

    switch (rxHeader.StdId) {
    case CAN_3510Moto1_ID:
    case CAN_3510Moto2_ID:
    case CAN_3510Moto3_ID:
    case CAN_3510Moto4_ID:
    {
        uint8_t i = (uint8_t)(rxHeader.StdId - CAN_3510Moto1_ID);
        if (moto_chassis[i].msg_cnt++ <= 50) {
            get_moto_offset(&moto_chassis[i], hcan, &rxHeader, rxData);
        } else {
            get_moto_measure(&moto_chassis[i], hcan, &rxHeader, rxData);
        }
        /* Optional mirror copy */
        get_moto_measure(&moto_info, hcan, &rxHeader, rxData);
        break;
    }
    default:
        break;
    }
}

/* Data parsing (standard RM/C610/C620 payload format):
   [0..1]=encoder (0..8191), [2..3]=speed_rpm (int16), [4..5]=iq_counts (int16), [6]=temp */
void get_moto_measure(moto_measure_t *ptr,
                      CAN_HandleTypeDef* hcan,
                      const CAN_RxHeaderTypeDef* hdr,
                      const uint8_t data[8])
{
    (void)hcan; (void)hdr;

    ptr->last_angle = ptr->angle;

    ptr->angle         = (uint16_t)((data[0] << 8) | data[1]);     // 0..8191
    ptr->speed_rpm     = (int16_t)((data[2] << 8) | data[3]);      // speed (rpm)
    ptr->given_current = (int16_t)((data[4] << 8) | data[5]);      // current (raw counts)
    ptr->real_current  = ptr->given_current;                       // alias for backward compatibility
    ptr->hall          = data[6];

    /* Cross-turn detection: maintain round_cnt */
    int32_t delta = (int32_t)ptr->angle - (int32_t)ptr->last_angle;
    if (delta > 4096)       ptr->round_cnt--;   /* reverse across 0 → 8191 */
    else if (delta < -4096) ptr->round_cnt++;   /* forward across 8191 → 0 */

    /* Accumulated angle: N*8192 + (angle - offset) */
    ptr->total_angle = ptr->round_cnt * 8192
                     + (int32_t)ptr->angle
                     - (int32_t)ptr->offset_angle;
}

/* Learn zero-offset during the first 50 frames */
void get_moto_offset(moto_measure_t *ptr,
                     CAN_HandleTypeDef* hcan,
                     const CAN_RxHeaderTypeDef* hdr,
                     const uint8_t data[8])
{
    (void)hcan; (void)hdr;
    ptr->angle = (uint16_t)((data[0] << 8) | data[1]);
    ptr->offset_angle = ptr->angle;
}

/* Transmit four motor currents (StdId 0x200, DLC=8; order is fixed: iq1→0x201, iq2→0x202, iq3→0x203, iq4→0x204) */
void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8];
    uint32_t mailbox;

    tx.StdId = 0x200;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    d[0] = (uint8_t)(iq1 >> 8);
    d[1] = (uint8_t)(iq1);
    d[2] = (uint8_t)(iq2 >> 8);
    d[3] = (uint8_t)(iq2);
    d[4] = (uint8_t)(iq3 >> 8);
    d[5] = (uint8_t)(iq3);
    d[6] = (uint8_t)(iq4 >> 8);
    d[7] = (uint8_t)(iq4);

    (void)HAL_CAN_AddTxMessage(hcan, &tx, d, &mailbox);
}

/* Convenience start (optional; usually unused in Plan A where main.c does the start) */
void BSP_CAN_Start_IT(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_Start(hcan);
}


