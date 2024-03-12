#include "bsp_spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

void SPI2_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi2.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&hspi2);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi2_rx);
    
    while(hdma_spi2_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_LISR_TCIF2);

    hdma_spi2_rx.Instance->PAR = (uint32_t) & (SPI2->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi2_rx.Instance->M0AR = (uint32_t)(rx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi2_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx, DMA_IT_TC);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi2_tx);
    
    while(hdma_spi2_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_LISR_TCIF3);

    hdma_spi2_tx.Instance->PAR = (uint32_t) & (SPI2->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi2_tx.Instance->M0AR = (uint32_t)(tx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi2_tx, num);
}

void SPI2_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    __HAL_DMA_DISABLE(&hdma_spi2_rx);
    __HAL_DMA_DISABLE(&hdma_spi2_tx);


    while(hdma_spi2_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_rx);
    }
    while(hdma_spi2_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi2_tx);
    }

    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx,__HAL_DMA_GET_DME_FLAG_INDEX(hspi2.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi2.hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx,__HAL_DMA_GET_DME_FLAG_INDEX(hspi2.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi2.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi2.hdmatx));


    hdma_spi2_rx.Instance->M0AR = rx_buf;
    hdma_spi2_tx.Instance->M0AR = tx_buf;

    __HAL_DMA_SET_COUNTER(&hdma_spi2_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi2_tx, ndtr);

    __HAL_DMA_ENABLE(&hdma_spi2_rx);
    __HAL_DMA_ENABLE(&hdma_spi2_tx);
}