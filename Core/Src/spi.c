/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
/* SPI1 错误码：在中断回调里只记录，由主循环打印，避免中断中做阻塞串口输出 */
volatile uint32_t spi1_error_code = 0U;
/* 编码器 CRC 失败计数：主循环打印，用来判断 SPI 数据是否有效 */
volatile uint32_t mt6701_crc_error_count = 0U;

/* 调试开关：1=校验 MT6701 CRC；0=跳过 CRC（仅用于对比 ADC ISR 耗时，量产要恢复 1） */
#define MT6701_CHECK_CRC 1
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Channel2;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <stdint.h>
#include <stdio.h>
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "arm_math.h"
#include "stm32f1xx_hal_spi.h"

uint8_t mt6701_rx_data[3];

uint8_t calculate_crc(uint32_t data)
{
  uint8_t crc = 0;
  uint32_t polynomial = 0x43; // (X^6 + X + 1)的二进制值

  // (D[13:0] 和 Mg[3:0])
  for (int i = 17; i >= 0; i--)
  {
    uint8_t bit = (data >> i) & 1;
    crc <<= 1;
    if ((crc >> 6) ^ bit)
      crc ^= polynomial;
    crc &= 0x3F;
  }
  return crc;
}

void mt6701_spi_dma_init(void)
{
  SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

  DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel2->CMAR = (uint32_t)mt6701_rx_data;
  DMA1_Channel2->CCR = DMA_CCR_MINC;

  DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel3->CMAR = (uint32_t)mt6701_rx_data;
  DMA1_Channel3->CCR = DMA_CCR_DIR;

  (void)SPI1->DR;
  (void)SPI1->SR;

  SPI1->CR1 |= SPI_CR1_SPE;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void mt6701_spi_start_tx(void)
{
  SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

  DMA1_Channel2->CNDTR = sizeof(mt6701_rx_data);
  DMA1_Channel3->CNDTR = sizeof(mt6701_rx_data);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  DMA1_Channel2->CCR |= DMA_CCR_EN;
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}

void mt6701_start_transfer(void)
{
  /* 保留原接口，内部改为低层 DMA，不再经过 HAL SPI */
  mt6701_spi_start_tx();
}

/* 在 ADC 中断里解析编码器原始数据（整数；CRC 可通过宏开关） */
void mt6701_update_angle(void)
{
  int angle_raw = (mt6701_rx_data[1] >> 2) | (mt6701_rx_data[0] << 6);

#if MT6701_CHECK_CRC
  uint8_t crc_raw = mt6701_rx_data[2] & ((1 << 6) - 1);
  uint32_t crc_data = ((uint32_t)mt6701_rx_data[0] << 16 | (uint32_t)mt6701_rx_data[1] << 8 | mt6701_rx_data[2]) >> 6;
  if (calculate_crc(crc_data) != crc_raw)
  {
    mt6701_crc_error_count++;
    return;
  }
#endif

  encoder_angle_hex = angle_raw;
  rotor_logic_angle_hex_norm = (rotor_logic_angle_hex & ((1 << ENCODER_BITS) - 1)) >> (ENCODER_BITS - sin_table_bit);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  spi1_error_code = hspi->ErrorCode;
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  }
  return;
}

/* USER CODE END 1 */

