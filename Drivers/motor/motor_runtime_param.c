#include "motor_runtime_param.h"

float motor_u_alpha;
float motor_u_beta;

float motor_i_u;
float motor_i_v;
float motor_i_alpha;
float motor_i_beta;
float motor_i_d;
float motor_i_q;

float encoder_speed;
float sensorless_speed_target_norm;
float motor_speed;
float motor_logic_angle;
float encoder_angle;
float encoder_init_angle;
float rotor_zero_angle;

int encoder_angle_hex;
int rotor_zero_angle_hex;
int rotor_logic_angle_hex_norm;