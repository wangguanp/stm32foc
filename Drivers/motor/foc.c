#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include "my_math.h"
#include "sin_table.h"
#include <stdbool.h>
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) ((180 * (a)) / PI)
#define rad60 deg2rad(60)

static arm_pid_instance_f32 pid_position;
static arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;

/* 闭控制上下文：foc.h 里是 extern 声明，这里必须有唯一定义 */
motor_control_context_t motor_control_context;

/* 串级中间指令 */
volatile float foc_speed_cmd = 0.0f;
volatile float foc_torque_cmd = 0.0f;

/* 环路输出与输入路由 */
float position_loop_out = 0.0f;
float speed_loop_out = 0.0f;
float torque_loop_d_out = 0.0f;
float torque_loop_q_out = 0.0f;

static float torque_d_const = 0.0f;

float *speed_loop_input = 0;
float *torque_loop_d_input = &torque_d_const;
float *torque_loop_q_input = &torque_d_const;
float *foc_forward_d_input = &torque_d_const;
float *foc_forward_q_input = &torque_d_const;
float *speed_loop_limit = 0;
float *torque_loop_q_limit = 0;

static void reset_all_pid(void)
{
  arm_pid_reset_f32(&pid_position);
  arm_pid_reset_f32(&pid_speed);
  arm_pid_reset_f32(&pid_torque_d);
  arm_pid_reset_f32(&pid_torque_q);
}

void motor_mode_position(float position)
{
  motor_control_context.type = control_type_position;
  motor_control_context.position = position;
  speed_loop_input = 0;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &torque_d_const;
  foc_forward_d_input = &torque_d_const;
  foc_forward_q_input = &position_loop_out;
  speed_loop_limit = 0;
  torque_loop_q_limit = 0;
  reset_all_pid();
}

void motor_mode_speed(float speed)
{
  motor_control_context.type = control_type_speed;
  motor_control_context.speed = speed;
  speed_loop_input = &motor_control_context.speed;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &torque_d_const;
  foc_forward_d_input = &torque_d_const;
  foc_forward_q_input = &speed_loop_out;
  speed_loop_limit = 0;
  torque_loop_q_limit = 0;
  reset_all_pid();
}

void motor_mode_torque(float d_norm, float q_norm)
{
  motor_control_context.type = control_type_torque;
  motor_control_context.torque_norm_d = d_norm;
  motor_control_context.torque_norm_q = q_norm;
  speed_loop_input = 0;
  torque_loop_d_input = &motor_control_context.torque_norm_d;
  torque_loop_q_input = &motor_control_context.torque_norm_q;
  foc_forward_d_input = &torque_loop_d_out;
  foc_forward_q_input = &torque_loop_q_out;
  speed_loop_limit = 0;
  torque_loop_q_limit = 0;
  reset_all_pid();
}

void motor_mode_position_torque(float position, float q_norm)
{
  motor_control_context.type = control_type_position_torque;
  motor_control_context.position = position;
  motor_control_context.torque_norm_q = q_norm;
  motor_control_context.max_torque_norm = q_norm;
  speed_loop_input = 0;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &position_loop_out;
  foc_forward_d_input = &torque_loop_d_out;
  foc_forward_q_input = &torque_loop_q_out;
  speed_loop_limit = &motor_control_context.max_torque_norm;
  torque_loop_q_limit = 0;
  reset_all_pid();
}

void motor_mode_speed_torque(float speed, float q_norm)
{
  motor_control_context.type = control_type_speed_torque;
  motor_control_context.speed = speed;
  motor_control_context.torque_norm_q = q_norm;
  motor_control_context.max_torque_norm = q_norm;
  speed_loop_input = &motor_control_context.speed;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &speed_loop_out;
  foc_forward_d_input = &torque_loop_d_out;
  foc_forward_q_input = &torque_loop_q_out;
  speed_loop_limit = 0;
  torque_loop_q_limit = &motor_control_context.max_torque_norm;
  reset_all_pid();
}

void motor_mode_position_speed(float position, float speed)
{
  motor_control_context.type = control_type_position_speed;
  motor_control_context.position = position;
  motor_control_context.speed = speed;
  motor_control_context.max_speed = speed;
  speed_loop_input = &position_loop_out;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &torque_d_const;
  foc_forward_d_input = &torque_d_const;
  foc_forward_q_input = &speed_loop_out;
  speed_loop_limit = &motor_control_context.max_speed;
  torque_loop_q_limit = 0;
  reset_all_pid();
}

void motor_mode_position_speed_torque(float position, float speed, float q_norm)
{
  motor_control_context.type = control_type_position_speed_torque;
  motor_control_context.position = position;
  motor_control_context.speed = speed;
  motor_control_context.torque_norm_q = q_norm;
  motor_control_context.max_speed = speed;
  motor_control_context.max_torque_norm = q_norm;
  speed_loop_input = &position_loop_out;
  torque_loop_d_input = &torque_d_const;
  torque_loop_q_input = &speed_loop_out;
  foc_forward_d_input = &torque_loop_d_out;
  foc_forward_q_input = &torque_loop_q_out;
  speed_loop_limit = &motor_control_context.max_speed;
  torque_loop_q_limit = &motor_control_context.max_torque_norm;
  reset_all_pid();
}


static const float sin_table_60[6] = {
    0.0000000000f,
    0.8660254038f,
    0.8660254038f,
    0.0000000000f,
    -0.8660254038f,
    -0.8660254038f};

static const float cos_table_60[6] = {
    1.0000000000f,
    0.5000000000f,
    -0.5000000000f,
    -1.0000000000f,
    -0.5000000000f,
    0.5000000000f};

static void svpwm(uint16_t phi_hex, float d, float q, float *d_u, float *d_v, float *d_w)
{
  d = clamp(d, -0.9f, 0.9f);
  q = clamp(q, -0.9f, 0.9f);

  const int v[6][3] = {{1,0,0},{1,1,0},{0,1,0},{0,1,1},{0,0,1},{1,0,1}};
  const int K_to_sector[] = {4,6,5,5,3,1,2,2};
  float sin_phi = sin_table(phi_hex);
  float cos_phi = cos_table(phi_hex);
  float alpha = 0;
  float beta = 0;
  arm_inv_park_f32(d,q,&alpha,&beta,sin_phi,cos_phi);

  bool A = beta > 0;
  bool B = fabsf(beta) > SQRT3 * fabsf(alpha);
  bool C = alpha > 0;

  int K = (A << 2) | (B << 1) | C;
  int sector = K_to_sector[K];

  float t_m = sin_table_60[sector % 6] * alpha - cos_table_60[sector % 6] * beta;
  float t_n = beta * cos_table_60[sector - 1] - alpha * sin_table_60[sector - 1];
  float t_0 = 1.0f - t_m - t_n;

  *d_u = (v[sector - 1][0] ? t_m : 0) + (v[sector % 6][0] ? t_n : 0) + t_0 / 2;
  *d_v = (v[sector - 1][1] ? t_m : 0) + (v[sector % 6][1] ? t_n : 0) + t_0 / 2;
  *d_w = (v[sector - 1][2] ? t_m : 0) + (v[sector % 6][2] ? t_n : 0) + t_0 / 2;
}

__attribute__((weak)) void set_pwm_duty(float d_u,float d_v,float d_w)
{
  while (1)
  {
    ;/* code */
  }
  
}

void foc_forward(float d,float q,int rotor_hex)
{
  float d_u = 0;
  float d_v = 0;
  float d_w = 0;
  svpwm((uint16_t)rotor_hex,d,q,&d_u,&d_v,&d_w);
  set_pwm_duty(d_u,d_v,d_w);

}

float cycle_diff(float diff, float cycle)
{
  if (diff > (cycle / 2))
    diff -= cycle;
  else if (diff < (-cycle / 2))
    diff += cycle;
  return diff;
}

//位置环
float position_loop(float position_ref, float position_fdb)
{
  float diff = cycle_diff(position_ref - position_fdb, position_cycle);
  float out = arm_pid_f32(&pid_position, diff);
  if (speed_loop_limit)
    out = clamp(out, -*speed_loop_limit, *speed_loop_limit);
  else
    out = clamp(out, -1.0f, 1.0f);
  foc_speed_cmd = out;
  return out;
}
void lib_position_control(float rad)
{
  float d = 0;
  float q = position_loop(rad, motor_logic_angle);
  foc_forward(d,q,rotor_logic_angle_hex_norm);
}

//速度环
float speed_loop(float speed_ref, float speed_fdb)
{
  float diff = speed_ref - speed_fdb;
  float out = arm_pid_f32(&pid_speed, diff);
  if (torque_loop_q_limit)
    out = clamp(out, -*torque_loop_q_limit, *torque_loop_q_limit);
  else
    out = clamp(out, -1.0f, 1.0f);
  pid_speed.state[2] = out;            // 积分反算抗饱和
  foc_torque_cmd = out;
  return out;
}
void lib_speed_control(float speed)
{
  float d = 0;
  float q = speed_loop(speed, motor_speed);
  foc_forward(d,q,rotor_logic_angle_hex_norm);
}

//转矩环：入参反馈已经归一化到 0~1
float torque_d_loop(float d_ref, float d_fdb)
{
  float diff = d_ref - d_fdb;
  float out = arm_pid_f32(&pid_torque_d, diff);
  out = clamp(out, -1.0f, 1.0f);       // 输出限幅
  pid_torque_d.state[2] = out;         // 积分反算抗饱和
  return out;
}
float torque_q_loop(float q_ref, float q_fdb)
{
  float diff = q_ref - q_fdb;
  float out = arm_pid_f32(&pid_torque_q, diff);
  out = clamp(out, -1.0f, 1.0f);       // 输出限幅
  pid_torque_q.state[2] = out;         // 积分反算抗饱和
  return out;
}

void lib_torque_control(float torque_norm_d,float torque_norm_q)
{
  float d = torque_d_loop(torque_norm_d, motor_i_d / MAX_CURRENT);
  float q = torque_q_loop(torque_norm_q, motor_i_q / MAX_CURRENT);
  foc_forward(d,q,rotor_logic_angle_hex_norm);
}
void lib_speed_torque_control(float speed_rad,float max_torque_norm)
{
  float torque_norm = speed_loop(speed_rad, motor_speed);
  torque_norm = min(fabs(torque_norm),max_torque_norm) * (torque_norm > 0 ? 1 : -1);
  foc_torque_cmd = torque_norm;          /* 速度环输出：力矩指令 */
  lib_torque_control(0,torque_norm);

}
void lib_position_speed_torque_control(float position_rad,float max_speed_rad,float max_torque_norm)
{
  float speed_rad = position_loop(position_rad, motor_logic_angle);
  speed_rad = min(fabs(speed_rad),max_speed_rad) * (speed_rad > 0 ? 1 : -1);
  foc_speed_cmd = speed_rad;             /* 位置环输出：速度指令 */
  lib_speed_torque_control(speed_rad,max_torque_norm);

}


void set_motor_pid(
  float position_p, float position_i,float position_d,
  float speed_p, float speed_i,float speed_d,
  float torque_d_p, float torque_d_i,float torque_d_d,
  float torque_q_p, float torque_q_i,float torque_q_d)
{
  pid_position.Kp = position_p;
  pid_position.Ki = position_i;
  pid_position.Kd = position_d;

  pid_speed.Kp = speed_p;
  pid_speed.Ki = speed_i;
  pid_speed.Kd = speed_d;

  pid_torque_d.Kp = torque_d_p;
  pid_torque_d.Ki = torque_d_i;
  pid_torque_d.Kd = torque_d_d;

  pid_torque_q.Kp = torque_q_p;
  pid_torque_q.Ki = torque_q_i;
  pid_torque_q.Kd = torque_q_d;

  arm_pid_init_f32(&pid_position,false); //falsed代表清空内部增量数据
  arm_pid_init_f32(&pid_speed,false);
  arm_pid_init_f32(&pid_torque_d,false);
  arm_pid_init_f32(&pid_torque_q,false);
}