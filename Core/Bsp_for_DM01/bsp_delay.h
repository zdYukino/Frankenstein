#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <sys/_stdint.h>

extern void DWT_Init();
extern void DWT_DelayUS(uint32_t _ulDelayTime);
extern void DWT_DelayMS(uint32_t _ulDelayTime);
#endif

