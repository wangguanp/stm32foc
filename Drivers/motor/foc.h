#pragma once
#include <stdbool.h>

void foc_forward(float d,float q,int rotor_hex);
void set_pwm_duty(float d_u, float d_v, float d_w);
float cycle_diff(float diff, float cycle);

float position_loop(float position_ref, float position_fdb);
float speed_loop(float speed_ref, float speed_fdb);
float torque_d_loop(float d_ref, float d_fdb);
float torque_q_loop(float q_ref, float q_fdb);

void lib_position_control(float rad);
void lib_speed_control(float speed);
void lib_torque_control(float torque_norm_d,float torque_norm_q);
void lib_speed_torque_control(float speed_rad,float max_torque_norm);
void lib_position_speed_torque_control(float position_rad,float max_speed_rad,float max_torque_norm);

typedef enum
{
    sensorless_control_state_null,
    sensorless_control_state_d,
    sensorless_control_state_openloop,
    sensorless_control_state_closeloop,
} sensorless_control_state_t;

typedef enum
{
    control_type_null,
    control_type_position,
    control_type_speed,
    control_type_torque,
    control_type_speed_torque,
    control_type_position_speed_torque,
    control_type_position_speed,
    control_type_position_torque,
    control_type_openloop,
} motor_control_type;

typedef struct
{
    motor_control_type type;
    float position;
    float speed;
    float torque_norm_d;
    float torque_norm_q;
    float max_speed;
    float max_torque_norm;
} motor_control_context_t;

extern motor_control_context_t motor_control_context;
extern sensorless_control_state_t sensorless_control_state;
extern bool is_sensorless;
extern int motor_dir;
extern float zero_const;

/* 环路输出与输入路由 */
extern float position_loop_out;
extern float speed_loop_out;
extern float torque_loop_d_out;
extern float torque_loop_q_out;

extern float *speed_loop_input;
extern float *torque_loop_d_input;
extern float *torque_loop_q_input;
extern float *foc_forward_d_input;
extern float *foc_forward_q_input;
extern float *speed_loop_limit;
extern float *torque_loop_q_limit;

/* 串级中间指令，便于 VOFA+ 观察 */
extern volatile float foc_speed_cmd;   /* 位置环输出（速度指令） */
extern volatile float foc_torque_cmd;  /* 速度环输出（力矩指令） */

/* 控制模式设置：设置输入源、限幅并复位 PID */
void motor_mode_position(float position);
void motor_mode_speed(float speed);
void motor_mode_torque(float d_norm, float q_norm);
void motor_mode_position_torque(float position, float q_norm);
void motor_mode_speed_torque(float speed, float q_norm);
void motor_mode_position_speed(float position, float speed);
void motor_mode_position_speed_torque(float position, float speed, float q_norm);
void motor_mode_off(void);

void set_motor_pid(
  float position_p, float position_i,float position_d,
  float speed_p, float speed_i,float speed_d,
  float torque_d_p, float torque_d_i,float torque_d_d,
  float torque_q_p, float torque_q_i,float torque_q_d);
