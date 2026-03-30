#ifndef _MY_FDCAN_H
#define _MY_FDCAN_H


#include "main.h"


uint32_t get_fdcan_dlc(uint16_t size);
uint16_t get_fdcan_data_size(uint32_t dlc);
void fdcan_filter_init(FDCAN_HandleTypeDef *fdcanHandle);
void fdcan_send(FDCAN_HandleTypeDef *fdcanHandle, uint32_t id, uint8_t *data, uint16_t size);



#endif
