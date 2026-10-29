#include "smo.h"
#include <math.h>
#include "my_math.h"
#include "arm_math.h"
#include "filter.h"
#include "sin_table.h"
#include "motor_runtime_param.h"
#include "openloop.h"
#include "stm32f1xx_hal.h"
#include "foc.h"

#define SMO_K (MOTOR_MAX_SPEED * POLE_PAIRS * PSI_F) // 覆盖住最大的反电动势=最大转速*磁链常数
#define SMO_SAT_A (MOTOR_MAX_CURRENT / 10.0f) // 用10%的最大电流

#define SMO_Z_LPF_ALPHA 0.33f
#define SMO_PLL_KP 128.0f
#define SMO_PLL_KI 30000.0f

#define SMO_PLL_OMEGA_MAX (MOTOR_MAX_SPEED * POLE_PAIRS)

/* 例程只在开环速度达到阈值时切换，不做 SMO 收敛/角度等待。 */

float smo_tilde_i_alpha;
float smo_tilde_i_beta;
float smo_hat_i_alpha;
float smo_hat_i_beta;
float smo_hat_theta_e;
float smo_hat_speed_e;
float smo_hat_e_alpha;
float smo_hat_e_beta;
float smo_hat_e_d;
float smo_hat_e_q;

float smo_pll_err;
float smo_pll_integral;

float smo_z_alpha;
float smo_z_beta;

float smo_k = SMO_K;
float smo_sat_a = SMO_SAT_A;
float smo_z_lpf_alpha = SMO_Z_LPF_ALPHA;
float smo_pll_kp = SMO_PLL_KP;
float smo_pll_ki = SMO_PLL_KI;

static float smo_i_decay;
static float smo_i_drive_gain;

void smo_var_init()
{
    smo_i_decay = expf(-MOTOR_RS * motor_adc_dt / MOTOR_LS);
    smo_i_drive_gain = (1.0f - smo_i_decay) / MOTOR_RS;
}

// 此函数理论推导来自于【stm32无感FOC理论与实践：滑模观测器】【01 观测反电动势】
// https://blog.csdn.net/qq570437459/article/details/158383069
static void smo()
{
    // 对应公式\tilde{i}_\alpha = i_\alpha-\hat{i}_\alpha
    smo_tilde_i_alpha = motor_i_alpha - smo_hat_i_alpha;
    smo_tilde_i_beta = motor_i_beta - smo_hat_i_beta;

    // 对应公式z_\alpha = -K \cdot \text{sat}(\tilde{i}_\alpha)
    if (smo_tilde_i_alpha > smo_sat_a)
        smo_z_alpha = -smo_k;
    else if (smo_tilde_i_alpha < -smo_sat_a)
        smo_z_alpha = smo_k;
    else
        smo_z_alpha = -(smo_k / smo_sat_a) * smo_tilde_i_alpha;

    if (smo_tilde_i_beta > smo_sat_a)
        smo_z_beta = -smo_k;
    else if (smo_tilde_i_beta < -smo_sat_a)
        smo_z_beta = smo_k;
    else
        smo_z_beta = -(smo_k / smo_sat_a) * smo_tilde_i_beta;

    // 可以直接用sat函数，但是上述展开后能减少计算量
    // smo_z_alpha = -smo_k * sat(smo_tilde_i_alpha / smo_sat_a);
    // smo_z_beta = -smo_k * sat(smo_tilde_i_beta / smo_sat_a);

    // 对应步骤：精确离散化
    smo_hat_i_alpha = smo_i_decay * smo_hat_i_alpha + smo_i_drive_gain * (motor_u_alpha - smo_z_alpha);
    smo_hat_i_beta = smo_i_decay * smo_hat_i_beta + smo_i_drive_gain * (motor_u_beta - smo_z_beta);

    // 对应步骤将$z_\alpha$输入低通滤波得到并更新$\hat{e_\alpha}$
    smo_hat_e_alpha = low_pass_filter(smo_z_alpha, smo_hat_e_alpha, smo_z_lpf_alpha);
    smo_hat_e_beta = low_pass_filter(smo_z_beta, smo_hat_e_beta, smo_z_lpf_alpha);
}

// 此函数理论推导来自于【stm32无感FOC理论与实践：滑模观测器】【02 PLL】
// https://blog.csdn.net/qq570437459/article/details/160866515
void smo_pll()
{
    smo();

    int hat_theta_hex = rad_to_index(smo_hat_theta_e);
    float sin_theta = sin_table(hat_theta_hex);
    float cos_theta = cos_table(hat_theta_hex);

    arm_park_f32(smo_hat_e_alpha,
                 smo_hat_e_beta,
                 &smo_hat_e_d,
                 &smo_hat_e_q,
                 sin_theta,
                 cos_theta);

    // 对应公式 err = -e_d / (omega * psi_f)，和教程例程一致
    smo_pll_err = -smo_hat_e_d / (smo_hat_speed_e * PSI_F + 1e-6f); // 1e-6f 防止除以 0

    // 积分项，积分饱和clamp截断
    smo_pll_integral =
        clamp(smo_pll_integral + smo_pll_ki * smo_pll_err * motor_adc_dt, -SMO_PLL_OMEGA_MAX, SMO_PLL_OMEGA_MAX);
    smo_hat_speed_e = clamp(smo_pll_integral + smo_pll_kp * smo_pll_err, -SMO_PLL_OMEGA_MAX, SMO_PLL_OMEGA_MAX);

    // 角度周期化到0到2pi之间
    smo_hat_theta_e = wrap_0_2pi_fast(smo_hat_theta_e + smo_hat_speed_e * motor_adc_dt);
}

// 判断是否可以从开环切换到闭环观测器
bool smo_is_ready()
{
    /* 和例程一致：只判断开环速度是否达到切换阈值。 */
    return fabsf(openloop_speed_e) >= SMO_SWITCH_SPEED;
}

float smo_check_lose_e_d;
float smo_check_lose_e_q;
// 无感观测保护，防止观测器失踪后还在长时间运行电机
bool check_smo_lose()
{
    // 失踪检测实时性要求不高，给予低通滤波
    smo_check_lose_e_d = low_pass_filter(smo_hat_e_d, smo_check_lose_e_d, 0.005f);
    smo_check_lose_e_q = low_pass_filter(smo_hat_e_q, smo_check_lose_e_q, 0.005f);

    return fabsf(smo_check_lose_e_d) > 0.4f || fabsf(smo_check_lose_e_q) < (SMO_CHECK_READY_SPEED * PSI_F * 0.7f);
}
