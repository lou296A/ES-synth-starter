#ifndef __BSP_DMA_H__
#define __BSP_DMA_H__

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_dma.h>
#include <stm32l4xx_hal_dac.h>

#define DAC_DR (0x4000 7400)

DMA_HandleTypeDef DMA_HandleStructure;
DAC_HandleTypeDef DAC_HandleStructure;

void DMA_Configuration(void);
//void DMA1_Channel3_IRQHandler(void);
#endif