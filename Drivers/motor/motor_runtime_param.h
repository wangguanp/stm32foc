#pragma  once
#include "conf.h"
//encode角度是编码器的角度的，机械角度， phy是当前角度减去0位角度也是机械角度
//logic是电角度
//带hex的是编码器的14位的那个和精度相关的
//带norm的是换成11位的精度用于后续的查表
#define rotor_phy_angle (encoder_angle - rotor_zero_angle)
#define rotor_logic_angle (rotor_phy_angle * POLE_PAIRS)
#define rotor_phy_angle_hex (encoder_angle_hex - rotor_zero_angle_hex)
#define rotor_logic_angle_hex (rotor_phy_angle_hex * POLE_PAIRS)

extern float motor_u_alpha;
extern float motor_u_beta;

extern float motor_i_u;
extern float motor_i_v;
extern float motor_i_alpha;
extern float motor_i_beta;
extern float motor_i_d;
extern float motor_i_q;

extern float encoder_speed;
extern float sensorless_speed_target_norm;
extern float motor_speed;
extern float motor_logic_angle;
extern float encoder_angle;
extern float encoder_init_angle;
extern float rotor_zero_angle;

extern int encoder_angle_hex;  //14位原始角
extern int rotor_zero_angle_hex;  //零位原始角
extern int rotor_logic_angle_hex_norm;  //11位点角度索引
