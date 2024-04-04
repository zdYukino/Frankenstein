#ifndef BSP_SBUS_H
#define BSP_SBUS_H

#include "main.h"

extern uint16_t sbus_channel[16];
extern float rc_dead_band_limit(float input, float dead_line);
extern void SBUS_trans(const uint8_t *buffer, uint8_t length);
#endif


