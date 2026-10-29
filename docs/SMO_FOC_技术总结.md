# 基于 STM32F103 的无感 FOC 滑模观测器移植与调试总结

> 版本：v1.0  
> 日期：2026-10-27  
> 平台：STM32F103C8 + MT6701 + 12V 电机  
> 内容：编码器 FOC 基线、SMO 无感移植、开环到闭环切换、低速阶跃调试、性能验证

---

## 1. 项目背景与目标

本项目最初已有一套基于 MT6701 编码器的 FOC 控制工程，包含：

- 电流环、速度环、位置环
- SVPWM
- ADC 注入采样
- 编码器角度和速度反馈

在此基础上，目标是增加无感 SMO FOC：

1. 移植 SMO 电流观测器和 PLL；
2. 实现开环 IF 强拖启动；
3. 实现开环 IF 到 SMO 闭环的切换；
4. 解决切换堵转、抖动、低速阶跃失锁等问题；
5. 在无载条件下验证 80~120 rad/s 的稳定运行。

本项目的重点不是重新推导 SMO 理论，而是：

- 把教程/例程的 SMO 正确接入现有工程；
- 找到“代码看起来一样，但运行结果不同”的差异；
- 建立可复现的调试方法。

---

## 2. 硬件与软件平台

### 2.1 硬件

| 项目 | 配置 |
|---|---|
| MCU | STM32F103C8，Cortex-M3，72MHz |
| 电机 | 7 对极，12V |
| 功率级 | 三相 PWM 驱动 |
| 电流采样 | 双电阻/运放，ADC 注入采样 |
| 编码器 | MT6701，SPI，14bit |
| 调试串口 | USART2，115200 |

### 2.2 软件

| 项目 | 配置 |
|---|---|
| PWM | TIM1 中心对齐，repetition=8，9 次更新触发一次 ADC |
| ADC | ADC1/ADC2 注入同步采样，约 4.44kHz |
| 速度环 | 500Hz |
| 电流/控制 | ADC 中断内运行 |
| SMO | `Drivers/motor/smo.c` |
| 开环 | `Drivers/motor/openloop.c` |
| FOC | `Drivers/motor/foc.c` |
| 状态机 | `Core/Src/main.c` |

---

## 3. 整体控制架构

```text
                 ┌──────────────┐
                 │  ADC 4.44kHz │
                 │  i_u / i_v   │
                 └──────┬───────┘
                        │
                   Clarke
                        │
        ┌───────────────┴────────────────┐
        │                                │
   SMO 电流观测                       Park 变换
   i_hat / tilde_i                    i_d / i_q
        │                                │
   z = -K·sat(tilde_i)                   │
        │                                │
   LPF → e_alpha/beta                    │
        │                                │
      PLL → theta_hat / omega_hat        │
        │                                │
        └───────────────┬────────────────┘
                        │
              速度环（500Hz）
                        │
                   q 电压给定
                        │
                     SVPWM
```

### 3.1 开环阶段

```text
IF 电流闭环 + 开环角度
→ 拖动转子
→ SMO 全程运行
```

### 3.2 切换阶段

```text
开环速度 > SMO_CHECK_READY_SPEED
→ 强制把 SMO 状态对齐到开环状态
→ 进入速度闭环
```

### 3.3 闭环阶段

```text
SMO 角度用于 Park 变换
SMO 速度用于速度环反馈
速度环直接给 q 电压
```

---

## 4. 关键控制流程

### 4.1 上电初始化

1. 外设初始化；
2. `smo_var_init()` 计算离散化系数；
3. ADC 零漂校准；
4. PWM/ADC 启动；
5. d 轴强拖，记录编码器零位；
6. 设置开环 PID；
7. 进入开环 IF。

### 4.2 ADC 中断顺序

```c
// 电流采样
motor_i_u = u_1 - adc_offset_u;
motor_i_v = u_2 - adc_offset_v;
arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);

// SMO 全程运行
smo_pll();

// 开环或闭环角度
if (openloop)      theta = openloop_theta_e;
else if (closeloop) theta = smo_hat_theta_e;

// Park + 电流环 + SVPWM
```

### 4.3 开环到闭环切换

```c
smo_hat_e_d = 0;
smo_hat_e_q = motor_dir * SMO_CHECK_READY_SPEED * PSI_F;
smo_check_lose_e_d = smo_hat_e_d;
smo_check_lose_e_q = smo_hat_e_q;

smo_hat_theta_e = openloop_theta_e;
smo_hat_speed_e = openloop_speed_e;
smo_pll_integral = openloop_speed_e;

sensorless_control_state = closeloop;

set_motor_pid(...);
motor_mode_speed(...);
```

核心思想：

> 不是等 SMO 自己收敛，而是用开环已知的 θ/ω 覆盖 SMO 状态，给它一个正确起点。

---

## 5. 关键参数

### 5.1 电机与电路

| 参数 | 值 |
|---|---|
| 极对数 | 7 |
| 磁链 `PSI_F` | 0.00257 |
| 相电阻 `MOTOR_RS` | 5.5Ω |
| 相电感 `MOTOR_LS` | 0.00085H |
| 母线电压 `U_BUS` | 12V |
| 最大无感电流 | 1.0A |
| 最大机械速度 | 200 rad/s |

### 5.2 SMO/PLL

| 参数 | 值 |
|---|---|
| `SMO_K` | 200×7×0.00257 = 3.598 |
| `SMO_SAT_A` | 0.1 |
| `SMO_Z_LPF_ALPHA` | 0.33 |
| `SMO_PLL_KP` | 128 |
| `SMO_PLL_KI` | 30000 |
| `SMO_PLL_OMEGA_MAX` | 1400 rad/s 电角度 |
| `SMO_CHECK_READY_SPEED` | 350 rad/s 电角度 ≈ 50 rad/s 机械（失步保护用） |
| `SMO_SWITCH_SPEED` | 560 rad/s 电角度 ≈ 80 rad/s 机械（I/F→SMO 切换） |
| `SMO_EXIT_SPEED` | 490 rad/s 电角度 ≈ 70 rad/s 机械（SMO→I/F 回切） |
| 失步阈值 `e_d` | 0.4V |
| 失步阈值 `e_q` | 0.63V |

### 5.3 开环与速度环

| 参数 | 值 |
|---|---|
| 开环加速度 | `120×7 = 840 rad/s²` |
| IF 最大 q 电流 | 0.3A |
| 速度环频率 | 500Hz |
| 速度环 PID | 0.4 / 0.4 |
| 速度给定斜坡 | 0.0005 / 2ms ≈ 50 rad/s² |
| 初始闭环目标 | -100 rad/s |
| 阶跃表 | 当前 demo：`{100, 80, 70, 70}`，每 5s 一步；此前 80/100/120 组合也验证过 |

---

## 6. SMO 关键公式

### 6.1 电流误差

```text
tilde_i_alpha = i_alpha - i_hat_alpha
tilde_i_beta  = i_beta  - i_hat_beta
```

### 6.2 滑模控制量

```text
z_alpha = -K · sat(tilde_i_alpha / SAT_A)
z_beta  = -K · sat(tilde_i_beta  / SAT_A)
```

### 6.3 电流观测

```text
i_hat_alpha[k+1] = decay·i_hat_alpha[k] + gain·(u_alpha - z_alpha)
i_hat_beta[k+1]  = decay·i_hat_beta[k]  + gain·(u_beta  - z_beta)
```

其中：

```text
decay = exp(-Rs·dt/Ls)
gain  = (1 - decay)/Rs
```

### 6.4 反电动势与 PLL

```text
e_alpha = LPF(z_alpha)
e_beta  = LPF(z_beta)

err = -e_d / (omega_hat·PSI_F + ε)
PLL 积分 → omega_hat
角度积分 → theta_hat
```

---

## 7. 调试过程与关键问题

这一部分是项目的核心。整个调试过程不是线性完成的，而是经历了“开环正常 → 一进闭环就堵转 → 怀疑 SMO → 怀疑 PID → 重新对比例程 → 最终定位到切换和阶跃瞬态”的过程。

### 7.1 编码器 FOC 基线

最开始先保留原有编码器 FOC 作为基线：

- 编码器角度正常；
- 电流环、速度环可运行；
- PWM/ADC/SPI 链路正常；
- 有位置、速度、力矩模式。

这一步的意义是：

> 后面无感出问题时，有一个确定可用的对照系统，可以判断是“公共底层问题”还是“SMO 新增问题”。

### 7.2 SMO 接入后 ZA/ZB 长期饱和

现象：

```text
ZA/ZB ≈ ±3.598
SMO_W 异常
EA/EB 不干净
PLL_INT 饱和
```

一开始怀疑 SMO 公式、符号、PLL 参数。排查后发现：

- `SMO_K = 200×7×0.00257 = 3.598`，ZA/ZB 饱和是限幅值；
- `z` 是滑模切换量，瞬时饱和不一定代表模型错；
- 真正要看的量是：
  - `smo_hat_i` 是否跟踪 `motor_i`；
  - `smo_tilde_i` 是否长期超过 `SMO_SAT_A`；
  - `EA/EB` 是否有基波；
  - `SMO_TH` 是否跟 `ENC_E`。

经验：

> 不能只看 ZA/ZB 是否饱和，要看电流观测和最终角度/速度。

### 7.3 PLL 锁到 2 倍频 / SMO_W=-200

现象：

```text
SMO_W=-200
PLL_INT=-1399
```

原因：

- PLL 误差分母使用 `smo_hat_speed_e * PSI_F + 1e-6`；
- 启动时 speed=0，分母接近 0；
- 误差被放大，积分冲到限幅；
- SMO_W 看起来像 -200，实际是限幅，不是真实速度。

解决与认识：

- 低速分母保护、PLL 初始化、切换时强制对齐都可以避免启动饱和；
- 但例程的核心不是“让开环 SMO 一定收敛”，而是切换时直接覆盖状态。

### 7.4 等 SMO 收敛再切，结果一直不切

一开始的逻辑：

```text
等 SMO_W 收敛到 -80
等 SMO_TH≈ENC_E
连续稳定 500ms
再切
```

实际现象：

- 开环 SMO 经常在 -180/-200/2 倍频；
- 等不到理想状态；
- 偶尔切一次，切进去又失败。

关键转折：

> 例程不是等 SMO 收敛，而是开环到速度后，直接把 `openloop_theta_e/openloop_speed_e` 覆盖到 SMO 状态。

最终切换代码：

```c
smo_hat_theta_e = openloop_theta_e;
smo_hat_speed_e = openloop_speed_e;
smo_pll_integral = openloop_speed_e;
```

这相当于：

```text
不再让 PLL 从 0 慢慢猜；
直接告诉它：现在角度是 θ_ol，速度是 ω_ol。
```

### 7.5 力矩过渡导致无载电机冲到 -190

中间尝试过：

```text
切换后先保持开环 q 电流 500ms
再切速度环
```

现象：

```text
ST=3 TYPE=3
电机从 -80 加速到 -190
最后 check_smo_lose 触发
```

原因：

- 无载电机；
- `openloop_iq_ref` 保持 0.7A；
- 没有速度环限制；
- 电机持续加速。

解决：

- 去掉力矩过渡；
- 切换后直接进入速度环；
- `motor_mode_speed()` 接管；
- 重置 4 个 PID。

### 7.6 速度环串电流环导致振荡

例程的无感速度模式：

```c
foc_forward_d_input = &zero_const;
foc_forward_q_input = &speed_loop_out;
```

速度环直接输出 q 电压，不串电流环。

我们中间保留了电流环：

```text
速度环 → 电流环 → SVPWM
```

现象：

- 速度在 -80/-28/-148 之间摆；
- SMO_W 也跟着跳；
- PLL_INT 在 1×/2× 之间变化。

解决：

- 无感速度模式改成速度环直接给 q 电压；
- 不再串电流环。

经验：

> 有感 FOC 的串级结构和无感例程的速度模式不要混用；先按例程跑通，再决定是否加电流环。

### 7.7 50ms 打印造成的误判

最初用 50ms 打印 `ENC_E/SMO_TH/EA/EB`，看起来“角度随机、正弦不对”。

实际：

```text
电频率约 89Hz
电周期约 11.2ms
50ms 约 4.5 个电周期
```

结论：

- 50ms 严重混叠；
- 只能看速度/趋势，不能判断正弦和相位；
- `ENC_E` 和 `SMO_TH` 同一时刻的差值有效，但连续波形无效。

解决：

- ADC 4.44kHz 停止前缓冲；
- 512 点 × 14 通道；
- 约 115ms；
- 停止时打印 `TRACE_STOP reason=...`。

### 7.8 -80 阶跃时 CLEQ 掉到 0.63

现象：

```text
TGT=-80
SMO_W≈-125
slout=+1
cleq≈-0.67
```

一开始怀疑 -80 的 SMO 观测能力不足。

固定 -80 测量后发现：

```text
ENC_W≈-80.5
SMO_W≈-77.8
EV/ET≈1.12
CLEQ≈-1.217
```

固定 -100：

```text
ENC_W≈-100.3
SMO_W≈-100.85
EV/ET≈1.27
CLEQ≈-1.189
```

结论：

- -80 稳态 SMO 正常；
- `cleq≈-0.67` 出现在 100→80 的阶跃瞬态；
- 速度环输出饱和，SMO 被 q 电压冲击；
- 不是 SMO 幅值不够。

解决：

- 速度给定斜坡；
- `SENSORLESS_REF_RAMP_STEP = 0.0005`；
- 500Hz 下约 50 rad/s²；
- -100→-80 约 0.4s；
- 阶跃表 `{80,100,120,100}` 稳定运行。

### 7.9 编码器关掉后的 ENC_E=0

现象：

```text
ENC_E=0
ENC_W=0
SMO_W=-100
```

一度怀疑编码器丢失。

实际：

- 硬件编码器开关被关闭；
- `ENC_E=0/ENC_W=0` 是正常现象；
- 闭环速度环用 SMO 速度，不用编码器；
- 但做 SMO 对比时必须打开编码器，仅用于观测。

经验：

> 调试时先确认每个传感器是否真的在工作，避免把“人为关闭”当成故障。

## 8. 性能与测试结果

### 8.1 已验证工况

| 工况 | 结果 |
|---|---|
| 固定 -80 rad/s | 稳定 |
| 固定 -100 rad/s | 稳定 |
| 固定 -120 rad/s | 稳定 |
| -100/-120 长跑 | 5 次 × 2 分钟稳定 |
| 80/100/120/100 阶跃 + 斜坡 | 长时间运行稳定 |
| 编码器对比 | 仅用于观测，不参与闭环 |

### 8.2 关键测量数据

固定 -80：

```text
TGT   = -80
ENCW  = -80.49
SMOW  = -77.81
ET    = 1.448V
EV    = 1.623V
EV/ET = 1.121
CLEQ  = -1.217
```

固定 -100：

```text
TGT   = -100
ENCW  = -100.29
SMOW  = -100.85
ET    = 1.804V
EV    = 2.288V
EV/ET = 1.268
CLEQ  = -1.189
```

解释：

- `SMOW` 和 `ENCW` 接近，说明速度估计正常；
- `EV` 和 `ET` 同量级，说明幅值没有明显缺失；
- `CLEQ` 绝对值远大于 0.63，说明稳态失步余量足够；
- `EV/ET` 略大于 1，可能包含 PWM 纹波、LPF 相位和参数误差，不代表模型错误。

### 8.3 角度与速度

- `SMO_TH` 与 `ENC_E` 多数误差小于 0.2 rad；
- 个别采样点可到 0.3~0.4 rad；
- -80 与 -100 稳态下，SMO 速度误差约 3% 以内；
- 速度环在 500Hz 运行。

### 8.4 资源占用

最终版本（包含 trace 和 MEAS）：

```text
Code    ≈ 27KB
RO Data ≈ 25KB
RW Data ≈ 240B
ZI Data ≈ 16.4KB
Flash   ≈ 52.7KB / 64KB
RAM     ≈ 16.4KB + 1KB 栈
```

### 8.5 调试工具

- 50ms DBG 行：慢变量、目标、速度、角度；
- 1s `MEAS` 行：EV/ET/CLEQ 平均；
- 4.44kHz 停止前缓冲：512 点 × 14 通道；
- 约 115ms 历史数据；
- `TRACE_STOP reason`：区分 ED/EQ 触发；
- 可用于分析：
  - `ed/eq`
  - `cled/cleq`
  - `smow/encw`
  - `pllint`
  - `slout`
  - `ence/smoth`
  - `iq`
  - `tgt`
  - `evec`

### 8.6 未验证范围

必须明确写进文档：

- 未做效率、温升、带载极限测试；
- 未验证 80 rad/s 以下；
- 未验证 120 rad/s 以上；
- 未做全速域、四象限、发电制动；
- 未做长期可靠性/EMC；
- 结果是实验平台、无载条件下的工程结果。

---

## 9. 代码结构

```text
Core/Src/main.c
  状态机、开环→闭环切换、速度阶跃、MEAS

Core/Src/adc.c
  ADC 电流采样、SMO 调用、trace 缓冲

Core/Src/tim.c
  速度计算、500Hz 速度环、速度给定斜坡

Drivers/motor/smo.c/.h
  SMO 电流观测、滑模 z、PLL、ready、check_lose

Drivers/motor/openloop.c/.h
  IF 开环强拖

Drivers/motor/foc.c/.h
  Park/Clarke、SVPWM、速度环、电流环、PID

Drivers/motor/conf.h
  电机参数、SMO/PLL 参数、斜坡参数

Drivers/motor/motor_runtime_param.c/.h
  全局运行变量
```

---

## 10. 调试方法论总结

### 10.1 先建立基线

- 编码器 FOC 先稳定；
- 再开无感；
- 公共底层（PWM/ADC/SPI）先确认。

### 10.2 一次只改一个变量

不建议同时改：

```text
SMO_K
SMO_SAT_A
PLL_KP/KI
速度环 PID
切换阈值
```

否则失败后无法判断原因。

### 10.3 先抓数据，再调参数

- 50ms 只能看趋势；
- 4.44kHz 才能看瞬态；
- `EV/ET` 分幅值/角度；
- `CLEQ` 看失步余量；
- `slout` 看速度环是否饱和。

### 10.4 区分“稳态”和“瞬态”

- 固定 -80 稳定，不代表 100→80 阶跃稳定；
- 阶跃失败可能是速度环饱和，不是 SMO 稳态模型错；
- 斜坡是解决瞬态的有效手段。

### 10.5 不要把保护当问题

`check_smo_lose` 触发不是“保护太敏感”，而是它检测到了：

```text
|e_d| > 0.4
或
|e_q| < 0.63
```

先找为什么，再决定是否调整。

---

## 11. 局限与未来工作

### 11.1 当前局限

- 速度范围约 80~120 rad/s；
- 低速依赖开环 IF 强拖；
- 80 以下未验证；
- 无载为主；
- 无感速度反馈依赖 SMO；
- 位置控制仍依赖编码器。

### 11.2 未来方向

1. 带载测试与电流/温升评估；
2. 低速 I/F 与 SMO 混合；
3. 高频注入；
4. 电机参数辨识；
5. 速度环/电流环抗饱和优化；
6. 故障诊断与在线记录；
7. 更高转速和弱磁。

---

## 附录 A：trace 字段

| 字段 | 含义 | 缩放 |
|---|---|---|
| `ed` | SMO d 轴反电动势 | ×1000 |
| `eq` | SMO q 轴反电动势 | ×1000 |
| `cled` | 失步检测 d 轴滤波 | ×1000 |
| `cleq` | 失步检测 q 轴滤波 | ×1000 |
| `smow` | SMO 机械速度 | ×100 |
| `encw` | 编码器机械速度 | ×100 |
| `pllint` | PLL 积分 | ×10 |
| `slout` | 速度环输出 q 电压 | ×10000 |
| `ence` | 编码器电角度 | ×1000 |
| `smoth` | SMO 电角度 | ×1000 |
| `iq` | q 轴电流 | ×1000 |
| `pllerr` | PLL 误差 | ×1000 |
| `tgt` | 速度目标 | ×100 |
| `evec` | 反电动势矢量幅值 | ×1000 |

## 附录 B：一句话总结

> 这个项目的关键不是“SMO 公式会不会写”，而是“如何把观测器状态在正确时机交给闭环”，以及“如何用可复现的数据区分 SMO、速度环和切换瞬态的问题”。
