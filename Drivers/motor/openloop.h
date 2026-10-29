#pragma once
#include <stdint.h>

typedef enum
{
    openloop_type_VF,
    openloop_type_IF,
} openloop_type_t;

extern openloop_type_t openloop_type; 

extern float openloop_theta_e;        // 开环电角度
extern float openloop_speed_e;        // 开环电角速度
extern float openloop_speed_ref_e;    // I/F 目标电角速度
extern float openloop_vf_output;

extern float openloop_iq_ref;         // 开环q轴电流给定

void openloop_VF();
void openloop_IF();
