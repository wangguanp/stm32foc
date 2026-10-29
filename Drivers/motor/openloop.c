#include "openloop.h"
#include "conf.h"
#include "my_math.h"
#include "foc.h"

sensorless_control_state_t sensorless_control_state = sensorless_control_state_null;
bool is_sensorless = false;
int motor_dir = 1;
float zero_const = 0.0f;

void motor_mode_off(void)
{
    motor_control_context.type = control_type_null;
    set_pwm_duty(0.0f, 0.0f, 0.0f);
}

#define OPENLOOP_ACC_E (POLE_PAIRS * 120.0f)       // VF电角速度爬升速率，弧度每秒
#define OPENLOOP_MAX_SPEED_E (POLE_PAIRS * 90.0f) // VF最大电角速度

#define OPENLOOP_VF_K 2.0f // VF控制的K常数

#define OPENLOOP_IF_ACC 5.0f    // IF电流爬升速率，A/s
#define OPENLOOP_IF_IQ_MAX 0.5f // IF最大电流，A

openloop_type_t openloop_type = openloop_type_IF;

float openloop_theta_e; //开环转子电角度
float openloop_speed_e;
float openloop_speed_ref_e = 0.0f; //开环电角速度
float openloop_vf_output; 

float openloop_iq_ref; //IF的目标q轴电流

void openloop_VF()
{
    {
        float step = OPENLOOP_ACC_E * motor_adc_dt;
        if (openloop_speed_e < openloop_speed_ref_e)
        {
            openloop_speed_e += step;
            if (openloop_speed_e > openloop_speed_ref_e)
                openloop_speed_e = openloop_speed_ref_e;
        }
        else if (openloop_speed_e > openloop_speed_ref_e)
        {
            openloop_speed_e -= step;
            if (openloop_speed_e < openloop_speed_ref_e)
                openloop_speed_e = openloop_speed_ref_e;
        }
    }
    openloop_speed_e = clamp(openloop_speed_e, -OPENLOOP_MAX_SPEED_E, OPENLOOP_MAX_SPEED_E);

    openloop_theta_e += openloop_speed_e * motor_adc_dt; // 积分出转子角度
    openloop_theta_e = wrap_0_2pi_fast(openloop_theta_e);

    float K = 0;
    if (motor_dir == 1)
        K = OPENLOOP_VF_K;
    else
        K = -OPENLOOP_VF_K;

    openloop_vf_output = (K + PSI_F * openloop_speed_e) / (U_BUS / SQRT3); // u_q=K+\omega\psi_f

    foc_forward_d_input = &zero_const;
    foc_forward_q_input = &openloop_vf_output; // svpwm的输入设置为VF的输出
}

void openloop_IF()
{
    {
        float step = OPENLOOP_ACC_E * motor_adc_dt;
        if (openloop_speed_e < openloop_speed_ref_e)
        {
            openloop_speed_e += step;
            if (openloop_speed_e > openloop_speed_ref_e)
                openloop_speed_e = openloop_speed_ref_e;
        }
        else if (openloop_speed_e > openloop_speed_ref_e)
        {
            openloop_speed_e -= step;
            if (openloop_speed_e < openloop_speed_ref_e)
                openloop_speed_e = openloop_speed_ref_e;
        }
    }
    openloop_speed_e = clamp(openloop_speed_e, -OPENLOOP_MAX_SPEED_E, OPENLOOP_MAX_SPEED_E);

    openloop_theta_e += openloop_speed_e * motor_adc_dt; // 积分出转子角度
    openloop_theta_e = wrap_0_2pi_fast(openloop_theta_e);

    // IF目标电流逐渐爬升
    if (motor_dir == 1)
        openloop_iq_ref += (OPENLOOP_IF_ACC / MOTOR_MAX_CURRENT) * motor_adc_dt;
    else
        openloop_iq_ref -= (OPENLOOP_IF_ACC / MOTOR_MAX_CURRENT) * motor_adc_dt;

    openloop_iq_ref =
        clamp(openloop_iq_ref, -OPENLOOP_IF_IQ_MAX / MOTOR_MAX_CURRENT, OPENLOOP_IF_IQ_MAX / MOTOR_MAX_CURRENT);

    //IF固定目标电流
    // if (dir == 1)
    //     openloop_iq_ref = OPENLOOP_IF_IQ_MAX / MOTOR_MAX_CURRENT;
    // else
    //     openloop_iq_ref = -OPENLOOP_IF_IQ_MAX / MOTOR_MAX_CURRENT;

    torque_loop_d_input = &zero_const;
    torque_loop_q_input = &openloop_iq_ref;
    foc_forward_d_input = &torque_loop_d_out;
    foc_forward_q_input = &torque_loop_q_out;
}
