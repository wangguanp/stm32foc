#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "conf.h"

#define SMO_CHECK_READY_SPEED (POLE_PAIRS * 50.0f)
/* I/F -> SMO 切换速度：机械 70 rad/s；与 check_smo_lose 的 50 阈值分开 */
#define SMO_SWITCH_SPEED (POLE_PAIRS * 60.0f)
/* SMO -> I/F 回切阈值：机械 70 rad/s */
#define SMO_EXIT_SPEED (POLE_PAIRS * 70.0f)

extern float smo_tilde_i_alpha;
extern float smo_tilde_i_beta;
extern float smo_hat_i_alpha;
extern float smo_hat_i_beta;
extern float smo_hat_theta_e;
extern float smo_hat_speed_e;
extern float smo_z_alpha;
extern float smo_z_beta;
extern float smo_hat_e_alpha;
extern float smo_hat_e_beta;
extern float smo_hat_e_d;
extern float smo_hat_e_q;
extern float smo_hat_e_vec;
extern float smo_k;
extern float smo_sat_a;
extern float smo_z_lpf_alpha;
extern float smo_pll_kp;
extern float smo_pll_ki;
extern float smo_pll_err;
extern float smo_pll_integral;

extern float smo_check_lose_e_d;
extern float smo_check_lose_e_q;

void smo_var_init();
void smo_pll();
bool smo_is_ready();
bool check_smo_lose();
