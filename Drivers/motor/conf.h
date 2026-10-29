#pragma once

//电机物理参数
#define POLE_PAIRS 7
#define PSI_F 0.00257f       // 磁链
#define MOTOR_RS 5.5f        // 相电阻，Ω
#define MOTOR_LS 0.00085f    // 相电感（d/q 平均），H

//电机控制参数
#define MOTOR_MAX_CURRENT 1.0f  // SMO/无感使用的最大电流，A
#define MOTOR_MAX_SPEED 200.0f  // 软件限制最大机械速度，rad/s

//电路参数
#define R_SHUNT 0.02f   //电流采样电阻欧姆
#define OP_GAIN 50.0f    //运放放大电路
#define MAX_CURRENT 2.0f  //有霍尔/编码器模式最大q轴电流
#define U_BUS 12.0f       // 母线电压，V
#define ADC_REFERENCE_VOLT 3.3f  //电流采样adc参考电压
#define ADC_BITS 12  //ADC精度，bit
#define ENCODER_BITS 14  //磁编码器精度，bit
#define sin_table_bit 11 //正弦查表位数
#define sin_table_size (1 << sin_table_bit)

//单片机配置参数
#define motor_pwm_freq 40000
#define motor_pwm_repetition 8
#define motor_adc_freq (motor_pwm_freq / (motor_pwm_repetition + 1.0f))
#define motor_adc_dt (1.0f / motor_adc_freq)
#define motor_speed_calc_freq 500
/* 无感速度给定斜坡：每 2ms 最多变化 0.0005（归一化），约 5rad/s per 100ms */
#define SENSORLESS_REF_RAMP_STEP 0.0005f
/* TIM4: 72MHz / (PSC+1) = 72MHz / 7 = 10.2857MHz，即每个计数约 0.0972us */
#define TIM4_TICK_HZ (SystemCoreClock / 7.0f)

//软件参数
#define position_cycle (6.0f * 3.14159265358979f) //电机多圈周期，等于正半周期+负半周期

