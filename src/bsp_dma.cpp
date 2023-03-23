#include "bsp_dma.h"

void DMA_Configuration(void)
{
    //DMA_InitTypeDef DMA_InitStructure; 
    HAL_Init();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DAC1_CLK_ENABLE();

    ;
    DAC_HandleStructure.Instance = DAC1;

    DMA_HandleStructure.Instance = DMA1_Channel3;
    DMA_HandleStructure.Init.Direction = DMA_MEMORY_TO_PERIPH;
    DMA_HandleStructure.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_HandleStructure.Init.MemInc = DMA_MINC_ENABLE;
    DMA_HandleStructure.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    DMA_HandleStructure.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    DMA_HandleStructure.Init.Mode = DMA_CIRCULAR;
    DMA_HandleStructure.Init.Priority = DMA_PRIORITY_HIGH;
    DMA_HandleStructure.Init.Request = DMA_REQUEST_6;
    
    HAL_DMA_Init(&DMA_HandleStructure);
    HAL_DAC_Init(&DAC_HandleStructure);

    __HAL_DMA_ENABLE_IT(&DMA_HandleStructure, DMA_IT_TC | DMA_IT_HT);
    
}

void DMA1_Channel3_IRQHandler(void) {
    // Check if the transfer is complete
    if (__HAL_DMA_GET_FLAG(&DMA_HandleStructure, DMA_FLAG_TC3)) {
        
        __HAL_DMA_CLEAR_FLAG(&DMA_HandleStructure, DMA_FLAG_TC3);
    }

    // Check if the transfer is half-complete
    if (__HAL_DMA_GET_FLAG(&DMA_HandleStructure, DMA_FLAG_HT3)) {
        // Clear the transfer half-complete flag
        __HAL_DMA_CLEAR_FLAG(&DMA_HandleStructure, DMA_FLAG_HT3);

        // Handle the transfer half-complete event
        // ...
    }
}