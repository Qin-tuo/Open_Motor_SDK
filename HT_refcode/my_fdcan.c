#include "my_fdcan.h"


FDCAN_TxHeaderTypeDef TxHeader =
{
    .TxFrameType = FDCAN_DATA_FRAME,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_ON,
    .FDFormat = FDCAN_FD_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
};


uint32_t get_fdcan_dlc(uint16_t size)
{
    uint32_t fdcan_dlc = 0;

    if(size == 0)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_0;
    }
    else if(size <= 1)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_1;
    }
    else if(size <= 2)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_2;
    }
    else if(size <= 3)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_3;
    }
    else if(size <= 4)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_4;
    }
    else if(size <= 5)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_5;
    }
    else if(size <= 6)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_6;
    }
    else if(size <= 7)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_7;
    }
    else if(size <= 8)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_8;
    }
    else if(size <= 12)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_12;
    }
    else if(size <= 16)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_16;
    }
    else if(size <= 20)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_20;
    }
    else if(size <= 24)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_24;
    }
    else if(size <= 32)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_32;
    }
    else if(size <= 48)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_48;
    }
    else if(size <= 64)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_64;
    }
    return fdcan_dlc;
}


uint16_t get_fdcan_data_size(uint32_t dlc)
{
    uint16_t size = 0;

    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0:
        size = 0;
        break;
    case FDCAN_DLC_BYTES_1:
        size = 1;
        break;
    case FDCAN_DLC_BYTES_2:
        size = 2;
        break;
    case FDCAN_DLC_BYTES_3:
        size = 3;
        break;
    case FDCAN_DLC_BYTES_4:
        size = 4;
        break;
    case FDCAN_DLC_BYTES_5:
        size = 5;
        break;
    case FDCAN_DLC_BYTES_6:
        size = 6;
        break;
    case FDCAN_DLC_BYTES_7:
        size = 7;
        break;
    case FDCAN_DLC_BYTES_8:
        size = 8;
        break;
    case FDCAN_DLC_BYTES_12:
        size = 12;
        break;
    case FDCAN_DLC_BYTES_16:
        size = 16;
        break;
    case FDCAN_DLC_BYTES_20:
        size = 20;
        break;
    case FDCAN_DLC_BYTES_24:
        size = 24;
        break;
    case FDCAN_DLC_BYTES_32:
        size = 32;
        break;
    case FDCAN_DLC_BYTES_48:
        size = 48;
        break;
    case FDCAN_DLC_BYTES_64:
        size = 64;
        break;
    default:
        break;
    }

    return size;
}

void fdcan_filter_init(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (HAL_FDCAN_ConfigGlobalFilter(fdcanHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(
                fdcanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_FDCAN_ConfigTxDelayCompensation(fdcanHandle, fdcanHandle->Init.DataPrescaler * fdcanHandle->Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(fdcanHandle);

    if (HAL_FDCAN_Start(fdcanHandle) != HAL_OK)
    {
        Error_Handler();
    }
}


void fdcan_send(FDCAN_HandleTypeDef *fdcanHandle, uint32_t id, uint8_t *data, uint16_t size)
{
    TxHeader.Identifier = id;

    if(id > 0x7ff)
    {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    }
    else
    {

        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    TxHeader.DataLength = get_fdcan_dlc(size);
    HAL_FDCAN_AddMessageToTxFifoQ(fdcanHandle, &TxHeader, data);
}
