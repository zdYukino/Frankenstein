#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <main.h>

extern void SPI2_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI2_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
