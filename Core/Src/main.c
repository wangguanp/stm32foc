/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "motor/motor_runtime_param.h"
#include "motor/foc.h"
#include "motor/smo.h"
#include "motor/openloop.h"
#include "motor/my_math.h"
#include "global_def.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 1：无感 SMO 测试；0：保留原来的编码器 FOC */
#define SENSORLESS_TEST 1
/* 0：强制停在开环，不进入 SMO，用于隔离开环/切换问题；1：允许切 SMO */
#define SMO_CLOSEDLOOP_ENABLE 1
/* 诊断开关：1=切换时用编码器角度对齐；确认开环角度问题后改回 0 */
#define SMO_SWITCH_USE_ENCODER 0
/* 0=只做 I/F->SMO 基线；1=启用 SMO->I/F 回切做 A/B */
#define SMO_BIDIRECTIONAL_ENABLE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float sensorless_speed_ref_norm = 0.0f;
static uint32_t sensorless_close_tick = 0U;
static uint32_t last_sensorless_step_tick = 0U;
static uint8_t sensorless_step_index = 0U;
static uint32_t smo_exit_tick = 0U;
volatile uint8_t motor_stop_reason = 0U;
static float meas_ev_sum = 0.0f;
static float meas_et_sum = 0.0f;
static float meas_cleq_sum = 0.0f;
static float meas_smow_sum = 0.0f;
static float meas_encw_sum = 0.0f;
static uint16_t meas_n = 0U;
static uint32_t meas_tick = 0U;

extern uint8_t mt6701_rx_data[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_pwm_duty(float d_u,float d_v,float d_w)
{
  htim1.Instance->CCR1 = (uint32_t)(d_u * htim1.Instance->ARR);
  htim1.Instance->CCR2 = (uint32_t)(d_v * htim1.Instance->ARR);
  htim1.Instance->CCR3 = (uint32_t)(d_w * htim1.Instance->ARR);

  /* 和例程一致：每次 set_pwm_duty 都反算相电压，供 SMO 使用。 */
  float d_mid = (d_u + d_v + d_w) / 3.0f;
  float u_u = (d_u - d_mid) * U_BUS;
  float u_v = (d_v - d_mid) * U_BUS;
  arm_clarke_f32(u_u, u_v, &motor_u_alpha, &motor_u_beta);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    //串级控制pid参数（自己调的）
   set_motor_pid(
       40.0f, 0.0f, 4.0f,
       0.01f, 0.0002f, 0.0f,
       0.02f, 0.007f, 0.0f,
       0.02f, 0.007f, 0.0f);

  //单环PID
  // set_motor_pid(
  //     2.0f, 0.0f, 4.0f,
  //     0.007f, 0.003f, 0.0f,
  //     0.8f, 0.2f, 0.0f,
  //     0.8f, 0.2f, 0.0f);
    //串级PID
      // set_motor_pid(
      // 40.0f, 0.0f, 4.0f,
      // 0.03f, 0.001f, 0.0f,
      // 0.02f, 0.007f, 0.0f,
      // 0.02f, 0.007f, 0.0f);

  /* 使能 DWT 周期计数器，用于测量 ADC 中断耗时 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if SENSORLESS_TEST
  /* SMO 离散化参数初始化，必须在使用 smo_pll() 之前调用 */
  smo_var_init();
#endif

  mt6701_spi_dma_init();
  mt6701_start_transfer();
  HAL_Delay(10); // 延时一会，让角度变量被赋值，不然默认角度是0


  /* TIM2 先启动，由 TIM1 TRGO 复位；OC1 中断里启动 SPI */
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  /* TIM4 自由运行，用于测速 dt */
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_Base_Start_IT(&htim3);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  /* ADC 零漂校准：电机未通电时采样 512 次 */
  adc_offset_calibration();

  /* 上电时记录一次编码器角度（位置模式会用到） */
  encoder_init_angle = encoder_angle;

  /* d 轴强拖：基础矢量 1（U 相高，V/W 低），把转子拉到电角度 0 */
  set_pwm_duty(0.5, 0, 0);
  HAL_Delay(400);                       // 等转子稳定，时间不够可以加长

  /* 此时转子 d 轴与基础矢量 1 重合，记录编码器角度作为零位 */
  rotor_zero_angle = encoder_angle;
  rotor_zero_angle_hex = encoder_angle_hex;

  /* 松开电机 */
  set_pwm_duty(0, 0, 0);
  HAL_Delay(100);

#if SENSORLESS_TEST
  /* 无感开环 IF 强拖，SMO 全程在 ADC 中断里计算 */
  motor_dir = -1;
  is_sensorless = true;
  openloop_type = openloop_type_IF;
  openloop_theta_e = 0.0f;
  openloop_speed_e = 0.0f;
  sensorless_speed_target_norm = motor_dir * 100.0f / MOTOR_MAX_SPEED;
  sensorless_speed_ref_norm = sensorless_speed_target_norm;
  openloop_speed_ref_e = motor_dir * SMO_SWITCH_SPEED;
  openloop_iq_ref = 0.0f;

  /* 用当前编码器电角度初始化 SMO，避免 PLL 从 0 角度锁到反方向。 */
  smo_hat_theta_e = index_to_rad(rotor_logic_angle_hex_norm);
  smo_hat_speed_e = 0.0f;
  smo_pll_integral = 0.0f;
  smo_pll_err = 0.0f;
  smo_hat_e_alpha = 0.0f;
  smo_hat_e_beta = 0.0f;
  smo_hat_e_d = 0.0f;
  smo_hat_e_q = 0.0f;
  smo_hat_i_alpha = 0.0f;
  smo_hat_i_beta = 0.0f;
  smo_z_alpha = 0.0f;
  smo_z_beta = 0.0f;

  set_motor_pid(
      0.0f, 0.0f, 0.0f,
      1.0f, 0.2f, 0.0f,
      0.19f, 0.02f, 0.0f,
      0.19f, 0.02f, 0.0f);

  motor_control_context.type = control_type_openloop;
  sensorless_control_state = sensorless_control_state_openloop;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 模式示例：按需要取消注释（不要直接改 motor_control_context.type） */
  // motor_mode_torque(0.0f, 0.1f);
  // motor_mode_position(deg2rad(180));
  // motor_mode_speed(60.0f);
  // motor_mode_position_torque(deg2rad(90.0f), 0.3f);
  // motor_mode_position_speed(deg2rad(90.0f), 20.0f);
  // motor_mode_position_speed_torque(deg2rad(90.0f), 20.0f, 0.3f);

  /* 双闭环：速度 + 电流 */
  /* 速度阶跃测试：30 -> 100 -> 60 rad/s，每 5s 切换一次 */
  static const float speed_step_table[] = {30.0f, 100.0f, 60.0f};
  const uint8_t speed_step_count =
      (uint8_t)(sizeof(speed_step_table) / sizeof(speed_step_table[0]));

  if (!is_sensorless)
  {
    motor_mode_speed_torque(speed_step_table[0], 0.5f);
  }

  uint32_t last_print_tick = 0U;
  uint32_t last_dbg_tick = 0U;
  uint32_t last_step_tick = HAL_GetTick();
  uint8_t speed_step_index = 0U;
// ENC_E   : 编码器电角度（用于对比）
// SMO_TH  : SMO 估算电角度
// SMO_W   : SMO 估算机械速度
// OL_TH   : 开环电角度
// OL_W    : 开环机械速度
// ST      : 0=停止, 2=开环, 3=SMO闭环

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if SENSORLESS_TEST && SMO_CLOSEDLOOP_ENABLE
    /* 开环速度达到阈值后切换到 SMO 闭环。
       和例程一致：切换时把 SMO 角度/速度/PLL 积分对齐到开环状态，
       然后直接进入速度环，不做 500ms 开环 q 电流力矩过渡。 */
    float sensorless_target_mech = fabsf(sensorless_speed_target_norm * MOTOR_MAX_SPEED);
    if (sensorless_control_state == sensorless_control_state_openloop &&
        sensorless_target_mech >= (SMO_SWITCH_SPEED / POLE_PAIRS) &&
        smo_is_ready())
    {
      /* 完全按例程：切换时强制把 SMO 角度/速度/PLL 积分对齐到开环状态。 */

      smo_hat_e_d = 0.0f;
      smo_hat_e_q = motor_dir * SMO_SWITCH_SPEED * PSI_F;
      smo_check_lose_e_d = smo_hat_e_d;
      smo_check_lose_e_q = smo_hat_e_q;

#if SMO_SWITCH_USE_ENCODER
      smo_hat_theta_e = index_to_rad(rotor_logic_angle_hex_norm);
#else
      smo_hat_theta_e = openloop_theta_e;
#endif
      smo_hat_speed_e = openloop_speed_e;
      smo_pll_integral = openloop_speed_e;

      sensorless_control_state = sensorless_control_state_closeloop;
      sensorless_speed_ref_norm = sensorless_speed_target_norm;

      set_motor_pid(
          0.0f, 0.0f, 0.0f,
          0.4f, 0.4f, 0.0f,
          0.3f, 0.5f, 0.0f,
          0.3f, 0.5f, 0.0f);

      /* 例程用 motor_mode_speed 直接接管：重置 4 个 PID，q 由速度环自然输出。 */
      motor_mode_speed(sensorless_speed_ref_norm);

      sensorless_close_tick = HAL_GetTick();
      last_sensorless_step_tick = HAL_GetTick();
      sensorless_step_index = 0U;
    }

    /* 无感闭环速度阶跃：切换后 5s 开始，每 5s 在 -100/-80/-120/-100 之间切换。
       只改速度给定，不重置 PID。 */
    /* 统一目标速度：I/F 和 SMO 都从同一张表更新目标。 */
    if ((sensorless_control_state == sensorless_control_state_openloop ||
         sensorless_control_state == sensorless_control_state_closeloop) &&
        motor_control_context.type != control_type_null &&
        (HAL_GetTick() - last_sensorless_step_tick) >= 5000U)
    {
      static const float sensorless_speed_step_table[] = {100.0f, 80.0f,50.0f};
      const uint8_t sensorless_speed_step_count =
          (uint8_t)(sizeof(sensorless_speed_step_table) / sizeof(sensorless_speed_step_table[0]));

      last_sensorless_step_tick += 5000U;
      sensorless_step_index++;
      if (sensorless_step_index >= sensorless_speed_step_count)
        sensorless_step_index = 0U;

      float target_norm =
          motor_dir * sensorless_speed_step_table[sensorless_step_index] / MOTOR_MAX_SPEED;

      sensorless_speed_target_norm = target_norm;
      sensorless_speed_ref_norm = target_norm;
      if (sensorless_control_state == sensorless_control_state_openloop)
        motor_control_context.speed = target_norm;
    }

    /* I/F 目标跟随目标速度，但最高只到 SMO 切换点。 */
    if (sensorless_control_state == sensorless_control_state_openloop)
    {
      float target_mech = fabsf(sensorless_speed_target_norm * MOTOR_MAX_SPEED);
      if (target_mech > (SMO_SWITCH_SPEED / POLE_PAIRS))
        target_mech = SMO_SWITCH_SPEED / POLE_PAIRS;
      openloop_speed_ref_e = motor_dir * target_mech * POLE_PAIRS;
    }

    /* SMO -> I/F：目标降到 75 以下，200ms 后回切，I/F 目标跟随目标值。 */
    if (sensorless_control_state == sensorless_control_state_closeloop &&
        motor_control_context.type == control_type_speed)
    {
      float target_mech = fabsf(sensorless_speed_target_norm * MOTOR_MAX_SPEED);
      if (target_mech <= (SMO_EXIT_SPEED / POLE_PAIRS))
      {
        if (smo_exit_tick == 0U)
          smo_exit_tick = HAL_GetTick();
        if ((HAL_GetTick() - smo_exit_tick) >= 200U)
        {
          openloop_theta_e = smo_hat_theta_e;
          openloop_speed_e = smo_hat_speed_e;
          openloop_speed_ref_e = motor_dir * target_mech * POLE_PAIRS;

          /* 回切 I/F：恢复 I/F 电流环参数，并给固定 q 电流初值。 */
          set_motor_pid(
              0.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 0.0f,
              0.19f, 0.02f, 0.0f,
              0.19f, 0.02f, 0.0f);
          openloop_iq_ref = motor_dir * 0.5f / MOTOR_MAX_CURRENT;
          sensorless_control_state = sensorless_control_state_openloop;
          motor_control_context.type = control_type_openloop;
          speed_loop_input = 0;
          torque_loop_d_input = &zero_const;
          torque_loop_q_input = &openloop_iq_ref;
          foc_forward_d_input = &torque_loop_d_out;
          foc_forward_q_input = &torque_loop_q_out;
          smo_exit_tick = 0U;
        }
      }
      else
        smo_exit_tick = 0U;
    }
    if (sensorless_control_state == sensorless_control_state_closeloop &&
        (HAL_GetTick() - sensorless_close_tick) > 500U &&
        motor_control_context.type != control_type_null &&
        check_smo_lose())
    {
      uint8_t reason = 0U;
      if (fabsf(smo_check_lose_e_d) > 0.4f)
        reason |= 0x01U;
      if (fabsf(smo_check_lose_e_q) < (SMO_CHECK_READY_SPEED * PSI_F * 0.7f))
        reason |= 0x02U;

      motor_stop_reason = reason;

      motor_control_context.type = control_type_null;
      set_pwm_duty(0.0f, 0.0f, 0.0f);
      sensorless_control_state = sensorless_control_state_null;
      is_sensorless = false;

      /* 先关机，再把停止前 512 点缓冲打印出来。 */
      trace_freeze_and_dump(reason);
    }
#endif

    /* 有感速度阶跃：只改参考值，不重新初始化 PID */
    if (!is_sensorless && (HAL_GetTick() - last_step_tick) >= 10000U)
    {
      last_step_tick += 10000U;

      if ((uint8_t)(speed_step_index + 1U) < speed_step_count)
      {
        speed_step_index++;
        motor_control_context.speed = speed_step_table[speed_step_index];
      }
    }

    /* 调试行 50ms 一次；用于看趋势，电角度正弦仍建议用缓冲采样 */
    if ((HAL_GetTick() - last_dbg_tick) >= 50U)
    {
      last_dbg_tick = HAL_GetTick();

      if (spi1_error_code != 0U)
      {
        printf("SPI1 Error: 0x%08lX\n", (unsigned long)spi1_error_code);
        spi1_error_code = 0U;
      }

      printf("DBG: ST=%d TYPE=%d ENC_E=%.3f SMO_TH=%.3f SMO_W=%.2f ED=%.3f EQ=%.3f PLL_ERR=%.3f PLL_INT=%.1f EA=%.3f EB=%.3f ZA=%.3f ZB=%.3f OL_W=%.2f TGT_W=%.2f ENC_W=%.2f CRC=%lu\n",
             (int)sensorless_control_state,
             (int)motor_control_context.type,
             (double)index_to_rad(rotor_logic_angle_hex_norm),
             (double)smo_hat_theta_e,
             (double)(smo_hat_speed_e / POLE_PAIRS),
             (double)smo_hat_e_d,
             (double)smo_hat_e_q,
             (double)smo_pll_err,
             (double)smo_pll_integral,
             (double)smo_hat_e_alpha,
             (double)smo_hat_e_beta,
             (double)smo_z_alpha,
             (double)smo_z_beta,
             (double)(openloop_speed_e / POLE_PAIRS),
             (double)(is_sensorless ? (motor_control_context.speed * MOTOR_MAX_SPEED)
                                    : motor_control_context.speed),
             (double)encoder_speed,
             (unsigned long)mt6701_crc_error_count);
      /* 每秒打印一次 -80/-100 点的 SMO 幅值统计，不参与控制。 */
      if (sensorless_control_state == sensorless_control_state_closeloop &&
          motor_control_context.type == control_type_speed)
      {
        float evec = sqrtf(smo_hat_e_alpha * smo_hat_e_alpha +
                            smo_hat_e_beta * smo_hat_e_beta);
        float ethe = fabsf(encoder_speed) * POLE_PAIRS * PSI_F;

        meas_ev_sum += evec;
        meas_et_sum += ethe;
        meas_cleq_sum += smo_check_lose_e_q;
        meas_smow_sum += (smo_hat_speed_e / POLE_PAIRS);
        meas_encw_sum += encoder_speed;
        meas_n++;

        if ((HAL_GetTick() - meas_tick) >= 1000U && meas_n > 0U)
        {
          float ratio = (meas_et_sum > 0.01f) ? (meas_ev_sum / meas_et_sum) : 0.0f;
          printf("MEAS N=%u EV=%.3f ET=%.3f RATIO=%.3f CLEQ=%.3f SMOW=%.2f ENCW=%.2f TGT=%.2f\\n",
                 (unsigned)meas_n,
                 (double)(meas_ev_sum / meas_n),
                 (double)(meas_et_sum / meas_n),
                 (double)ratio,
                 (double)(meas_cleq_sum / meas_n),
                 (double)(meas_smow_sum / meas_n),
                 (double)(meas_encw_sum / meas_n),
                 (double)(motor_control_context.speed * MOTOR_MAX_SPEED));
          meas_ev_sum = 0.0f;
          meas_et_sum = 0.0f;
          meas_cleq_sum = 0.0f;
          meas_smow_sum = 0.0f;
          meas_encw_sum = 0.0f;
          meas_n = 0U;
          meas_tick = HAL_GetTick();
        }
      }
      else
      {
        meas_ev_sum = 0.0f;
        meas_et_sum = 0.0f;
        meas_cleq_sum = 0.0f;
        meas_smow_sum = 0.0f;
        meas_encw_sum = 0.0f;
        meas_n = 0U;
        meas_tick = HAL_GetTick();
      }

      tim2_oc_count = 0U;
      mt6701_crc_error_count = 0U;
      dbg_speed_loop_count = 0U;
    }

    /* CSV 保持 20ms 一次，恢复完整的 15 通道 */
    if ((HAL_GetTick() - last_print_tick) >= 20U)
    {
      last_print_tick = HAL_GetTick();

      float ref = 0.0f;
      switch (motor_control_context.type)
      {
      case control_type_position:
      case control_type_position_speed:
      case control_type_position_torque:
      case control_type_position_speed_torque:
        ref = motor_control_context.position;
        break;
      case control_type_speed:
      case control_type_speed_torque:
        ref = motor_control_context.speed;
        break;
      case control_type_torque:
        ref = motor_control_context.torque_norm_q;
        break;
      default:
        ref = 0.0f;
        break;
      }

      /* CSV: type,ref,Id,Iq,speed,encoder_angle,rotor_logic_angle,motor_logic_angle,pos_out,spd_out,isr_us,t_enc,t_conv,t_pid,t_foc */
      // printf("foc:%.0f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      //        (float)motor_control_context.type, //I0
      //        ref,  //I1
      //        motor_i_d,  //I2
      //        motor_i_q,  //I3
      //        motor_speed,  //I4
      //        encoder_angle,  //I5
      //        rotor_logic_angle,  //I6
      //        motor_logic_angle,  //I7
      //        foc_speed_cmd,  //I8
      //        foc_torque_cmd,  //I9
      //        adc_irq_cycles * (1000000.0f / (float)SystemCoreClock), //I10
      //        adc_t_enc  * (1000000.0f / (float)SystemCoreClock), //I11
      //        adc_t_conv * (1000000.0f / (float)SystemCoreClock), //I12
      //        adc_t_pid  * (1000000.0f / (float)SystemCoreClock), //I13
      //        adc_t_foc  * (1000000.0f / (float)SystemCoreClock)); //I14
    }
  }
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
