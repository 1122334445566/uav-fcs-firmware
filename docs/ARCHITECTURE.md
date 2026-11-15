# 架构设计

## 1. 设计目标

1. **硬件无关**：`src/` 下不允许出现任何厂商头文件、`HAL_GPIO_`、`printf`、`malloc`。
2. **确定性**：每个控制周期的计算量恒定，无动态内存、无阻塞、无锁。
3. **可验证**：同一份 `.c` 既能交叉编译进 MCU，也能在 PC 上链接模拟器跑闭环。
4. **安全隔离**：安全监控与控制器解耦，故障时可绕过控制器直接切断电机输出。

## 2. 分层

```
┌───────────────────────────────────────────────────────────────┐
│  应用层：main() / RTOS 任务 / 地面站                            │
│  只做三件事：构造 fcs_hal_t、调用 fcs_init()、周期调用 fcs_step()│
├───────────────────────────────────────────────────────────────┤
│  飞控核心（本仓库主体）                                        │
│  fcs.c 调度 → attitude / state_machine / failsafe /            │
│               controller / mixer / telemetry / params          │
├───────────────────────────────────────────────────────────────┤
│  HAL 抽象：fcs_hal_t 函数指针表（11 个回调）                   │
├──────────────────┬────────────────────────────────────────────┤
│  platform/stm32  │  platform/posix（SIL）                     │
│  真实 BSP         │  6-DOF 动力学 + 带噪声合成传感器            │
└──────────────────┴────────────────────────────────────────────┘
```

依赖方向严格单向：核心只依赖 `include/fcs/*.h` 与 C99 标准库（`<math.h>`、`<string.h>`、
`<stdio.h>` 仅用于 `vsnprintf` 日志格式化）。核心**不依赖** HAL 的实现。

## 3. 坐标系与单位

| 量 | 约定 |
| --- | --- |
| 世界系 | NED：x 北、y 东、z 下；原点为起飞点 |
| 机体系 | x 前、y 右、z 下（右手系） |
| 四元数 | `(q0,q1,q2,q3) = (w,x,y,z)`，单位四元数，把向量从**机体旋到 NED** |
| 欧拉角 | ZYX 顺序（yaw→pitch→roll），弧度；NED 下 pitch 正 = 机头上仰 |
| 高度 | 正向**向上**，等于 `-pos_ned.z` |
| 角速率 | 机体系 `(p,q,r)`，rad/s |
| 力矩/油门 | 归一化 `[-1,1]` / `[0,1]`，由 BSP 映射成 DShot/PWM |

所有模块都遵守同一约定；`fcs_quat_rotate(q, v)` = body→NED，
`fcs_quat_rotate_inv(q, v)` = NED→body。改约定必须同时改数学库、估计器、控制器、混控和仿真器。

## 4. 主循环时序

`fcs_step(f, now_us)` 是唯一的入口，`now_us` 必须单调递增：

| 步骤 | 模块 | 说明 |
| --- | --- | --- |
| 1 | `fcs.c` | 计算 `dt`，钳制到 `[0.2 ms, 50 ms]`，记录最大周期 |
| 2 | HAL | 非阻塞读 IMU / 气压 / GNSS / RC / 电池；无新样本返回 false，沿用旧状态 |
| 3 | `attitude.c` | Mahony 更新四元数；填充 `state.euler` / `state.rates` |
| 4 | `failsafe.c` | 6 条规则求值，返回"本 tick 新出现"的故障事件 |
| 5 | `state_machine.c` | 处理故障、解锁/上锁、模式切换、航点推进，产出 `setpoint` |
| 6 | `controller.c` | 串级 PID → `fcs_control_t{thrust, torque}`（MANUAL 直通） |
| 7 | `mixer.c` | 4×3 混控矩阵 + 姿态优先饱和 → 4 路 `[0,1]` |
| 8 | `fcs.c` | **二次互锁**：非 ARMED/LANDING 时清零全部电机 |
| 9 | HAL | `motor_write`、按分频发遥测、`watchdog_feed` |

设计要点：第 8 步的互锁与第 5 步的状态机是两处独立判断。即使控制器或状态机被
故障数据写坏，`fcs_sm_armed()` 返回 false 时电机依旧为零。

## 5. 故障传播

```
failsafe_check() 产生事件（只发一次）
        │
        ├─ KILL 类（ATTITUDE_UPSET / IMU_ERROR / LOOP_STALL）
        │       → 状态机立刻 DISARMED → 电机全零
        │
        ├─ LAND 类（RC_LOSS / BATTERY_CRITICAL）
        │       → LANDING：锁住水平位置，目标高度以 0.6 m/s 下降
        │       → 高度 < 0.15 m 且持续 1 s → DISARMED
        │       → 例外：AUTO 且任务进行中，RC_LOSS 继续执行任务
        │
        └─ DEGRADE 类（GNSS_LOSS）
                → 降级 ALT_HOLD + 置 gnss_degraded
                → RC 再请求 POS_HOLD/AUTO 会被拒绝
                → 核心收到一帧有效 GNSS 后自动解除
```

`fcs_failsafe_t` 不持有 `fcs_controller_t` 指针，也不调用任何控制函数，保证安全逻辑可独立审查。

## 6. 实时性与内存

- `fcs_t` 全部为值类型，约 1–2 KB，**静态分配**，无堆。
- 单周期运算：约 20 次四元数/向量运算 + 3 组 PID + 4 路混控；无矩阵求逆、无迭代、无 `sqrt` 密集调用。
- 建议在 Cortex-M4F@168 MHz 上 1 kHz 运行，SIL 验证使用 500 Hz（余量更大）。
- `fcs_step()` 最长执行路径与状态无关；`fcs_telemetry_*` 按 `telemetry_divider`（默认 50）分频，
  避免遥测阻塞控制周期。日志只在状态/模式/故障变化时产生。
- 浮点使用 `float`（单精度），与 MCU 的 FPU 一致；`double` 仅出现在 GNSS 经纬度字段。

## 7. 扩展点

| 想替换什么 | 改哪里 | 影响范围 |
| --- | --- | --- |
| 机型（六旋翼、固定翼） | `src/control/mixer.c` 的矩阵 + `FCS_MAX_MOTORS` | 仅混控与 SIL 动力学 |
| 姿态估计算法（EKF） | `src/estimation/attitude.c` | 仅估计器，接口不变 |
| 遥测协议（真 MAVLink） | `src/core/telemetry.c` | 仅编解码，HAL 的 `telemetry_write` 不变 |
| 控制律（LQR / ADRC） | `src/control/controller.c` | 仅控制器 |
| 板卡 | `platform/stm32/` 或新目录 | 核心零改动 |
| 参数存储（Flash/FRAM/EEPROM） | HAL 的 `param_load` / `param_save` | 核心零改动 |

## 8. 为什么遥测不是真 MAVLink

真 MAVLink 需要生成头文件、CRC extra、序列化生成器与 ~50 KB 代码。本项目用 60 行实现了
同构的帧结构和 CRC16-CCITT，便于完整展示"帧 → 校验 → 解析"链路且保持零依赖。
生产环境建议把 `telemetry.c` 换成官方 `mavlink_c_library_v2`，`fcs_telemetry_send_*`
的签名可以保持不变。
