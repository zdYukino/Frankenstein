#ifndef BSP_SBUS_H
#define BSP_SBUS_H

#include "main.h"

extern uint16_t sbus_channel[16];
extern void SBUS_trans(const uint8_t *buffer, uint8_t length);
#endif


