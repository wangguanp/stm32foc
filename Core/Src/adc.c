/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"
#include <stdio.h>
#include <math.h>

/* USER CODE BEGIN 0 */
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "motor/sin_table.h"
#include "motor/smo.h"
#include "motor/openloop.h"
#include "motor/my_math.h"
#include "filter.h"
#include "spi.h"
#include "global_def.h"
#include "arm_math.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 规则通道用于例程的阻塞式 ADC 零漂校准：ADC1 -> PA0。 */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 规则通道用于例程的阻塞式 ADC 零漂校准：ADC2 -> PA1。 */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA1     ------> ADC2_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC1:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA1     ------> ADC2_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
  /* USER CODE END ADC2:ADC1_2_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
float adc_offset_u = 0.0f;
float adc_offset_v = 0.0f;
volatile uint32_t adc_irq_cycles = 0;
volatile uint32_t adc_t_enc = 0;
volatile uint32_t adc_t_conv = 0;
volatile uint32_t adc_t_pid = 0;
volatile uint32_t adc_t_foc = 0;

/* 4.44kHz 停止前缓冲：512 点约 115ms。
   通道顺序和缩放见 trace_freeze_and_dump() 的表头。 */
#define TRACE_SAMPLES 512U
#define TRACE_CHANNELS 14U

typedef struct
{
  int16_t sample[TRACE_SAMPLES][TRACE_CHANNELS];
  volatile uint16_t head;
  volatile uint8_t frozen;
} TraceBuffer;

static TraceBuffer trace_buf;

static void trace_sample(void)
{
  if (trace_buf.frozen)
    return;

  int16_t *s = trace_buf.sample[trace_buf.head];

  s[0]  = (int16_t)(smo_hat_e_d * 1000.0f);
  s[1]  = (int16_t)(smo_hat_e_q * 1000.0f);
  s[2]  = (int16_t)(smo_check_lose_e_d * 1000.0f);
  s[3]  = (int16_t)(smo_check_lose_e_q * 1000.0f);
  s[4]  = (int16_t)((smo_hat_speed_e / POLE_PAIRS) * 100.0f);
  s[5]  = (int16_t)(encoder_speed * 100.0f);
  s[6]  = (int16_t)(smo_pll_integral * 10.0f);
  s[7]  = (int16_t)(speed_loop_out * 10000.0f);
  s[8]  = (int16_t)(index_to_rad(rotor_logic_angle_hex_norm) * 1000.0f);
  s[9]  = (int16_t)(smo_hat_theta_e * 1000.0f);
  s[10] = (int16_t)(motor_i_q * 1000.0f);
  s[11] = (int16_t)(smo_pll_err * 1000.0f);
  s[12] = (int16_t)((is_sensorless ? (motor_control_context.speed * MOTOR_MAX_SPEED)
                                    : motor_control_context.speed) * 100.0f);
  s[13] = (int16_t)(sqrtf(smo_hat_e_alpha * smo_hat_e_alpha + smo_hat_e_beta * smo_hat_e_beta) * 1000.0f);

  trace_buf.head++;
  if (trace_buf.head >= TRACE_SAMPLES)
    trace_buf.head = 0U;
}

void trace_freeze_and_dump(uint8_t reason)
{
  trace_buf.frozen = 1U;

  uint16_t idx = trace_buf.head;

  printf("TRACE_STOP reason=%u samples=%u dt_us=225\n",
         (unsigned)reason, (unsigned)TRACE_SAMPLES);
  printf("idx,ed,eq,cled,cleq,smow,encw,pllint,slout,ence,smoth,iq,pllerr,tgt,evec\n");
  printf("scale:ed/eq/cled/cleq/ence/smoth/iq/pllerr/evec=1000,smow/encw/tgt=100,pllint=10,slout=10000\n");

  for (uint16_t i = 0U; i < TRACE_SAMPLES; i++)
  {
    int16_t *s = trace_buf.sample[idx];

    printf("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           (unsigned)i,
           s[0], s[1], s[2], s[3], s[4], s[5], s[6],
           s[7], s[8], s[9], s[10], s[11], s[12], s[13]);

    idx++;
    if (idx >= TRACE_SAMPLES)
      idx = 0U;
  }
}

/* 例程式 ADC 零漂校准：电机未通电时用规则通道阻塞采样 512 次。 */
void adc_offset_calibration(void)
{
  const uint16_t n = 512;
  float sum_u = 0.0f;
  float sum_v = 0.0f;

  for (uint16_t i = 0; i < n; i++)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
    {
      sum_u += (float)HAL_ADC_GetValue(&hadc1) * (ADC_REFERENCE_VOLT / (1 << ADC_BITS)) -
               ADC_REFERENCE_VOLT / 2.0f;
    }

    if (HAL_ADC_PollForConversion(&hadc2, 10U) == HAL_OK)
    {
      sum_v += (float)HAL_ADC_GetValue(&hadc2) * (ADC_REFERENCE_VOLT / (1 << ADC_BITS)) -
               ADC_REFERENCE_VOLT / 2.0f;
    }
  }

  adc_offset_u = sum_u / (float)n;
  adc_offset_v = sum_v / (float)n;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    uint32_t irq_start = DWT->CYCCNT;

    /* 上一次 SPI 传输已完成，拉高 CS；下一次 TIM2 OC 会再拉低 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    mt6701_update_angle();
    uint32_t t_enc = DWT->CYCCNT;

    /* SPI 由 TIM2 的 OC1 中断启动，ADC 这里只解析上一帧数据 */

    float u_1 = (float)hadc1.Instance->JDR1 * (ADC_REFERENCE_VOLT / (1 << ADC_BITS)) - ADC_REFERENCE_VOLT / 2.0f;
    float u_2 = (float)hadc2.Instance->JDR1 * (ADC_REFERENCE_VOLT / (1 << ADC_BITS)) - ADC_REFERENCE_VOLT / 2.0f;

    /* 和例程一致：电流使用规则通道校准后的零漂补偿。 */
    motor_i_u = u_1 - adc_offset_u;
    motor_i_v = u_2 - adc_offset_v;

    float i_alpha = 0.0f;
    float i_beta = 0.0f;
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
    motor_i_alpha = i_alpha;
    motor_i_beta = i_beta;

    int theta_e_hex_norm = rotor_logic_angle_hex_norm;

    /* 和例程一致：SMO 从 ADC 启动开始全程运行，包括强拖和开环阶段。 */
    smo_pll();

    if (is_sensorless)
    {
      switch (sensorless_control_state)
      {
      case sensorless_control_state_openloop:
        if (openloop_type == openloop_type_VF)
          openloop_VF();
        else
          openloop_IF();
        theta_e_hex_norm = rad_to_index(openloop_theta_e);
        break;
      case sensorless_control_state_closeloop:
        theta_e_hex_norm = rad_to_index(smo_hat_theta_e);
        break;
      default:
        return;
      }
    }

    float sine_value = sin_table(theta_e_hex_norm);
    float cos_value = cos_table(theta_e_hex_norm);
    float _motor_i_d = 0;
    float _motor_i_q = 0;
    arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sine_value, cos_value);
    /* 例程 ADC 中不做 d/q 低通：省掉两次函数调用，也避免电流环相位滞后 */
    motor_i_d = _motor_i_d;
    motor_i_q = _motor_i_q;
    uint32_t t_conv = DWT->CYCCNT;
    //只要和位置环相关的模式式就运行位置环
    /* 位置环已在 TIM3 中计算 */
    //只要和速度环相关的就运行速度环
    /* 速度环已在 TIM3 中计算 */
    //电流环始终运行
    /* 和例程一致：电流反馈直接用安培，PID 增益含义不变 */
    torque_loop_d_out = torque_d_loop(*torque_loop_d_input, motor_i_d);
    torque_loop_q_out = torque_q_loop(*torque_loop_q_input, motor_i_q);
    uint32_t t_pid = DWT->CYCCNT;
    //SVPWM的输出
    if (motor_control_context.type != control_type_null)
    {
      foc_forward(*foc_forward_d_input, *foc_forward_q_input, theta_e_hex_norm);
    }

    uint32_t t_foc = DWT->CYCCNT;
    adc_t_enc  = t_enc  - irq_start;
    adc_t_conv = t_conv - t_enc;
    adc_t_pid  = t_pid  - t_conv;
    adc_t_foc  = t_foc  - t_pid;
    adc_irq_cycles = t_foc - irq_start;

    trace_sample();
   }
}
/* USER CODE END 1 */

