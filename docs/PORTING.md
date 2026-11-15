# 移植到 STM32（或其他 MCU）

核心库 `fcs_core` 只依赖 C99 + `<math.h>`。移植工作 = **实现 HAL + 挂接一个周期时钟**。
参考实现见 `platform/stm32/hal_stm32_template.c`。

## 1. 需要实现的回调

`fcs_hal_validate()` 要求 4 个**必需**回调：`imu_read`、`rc_read`、`motor_write`、`time_us`。
其余为可选（传 NULL 即可，核心会做空指针判断）。

| 回调 | 必需 | 要求 |
| --- | --- | --- |
| `imu_read` | ✅ | 非阻塞；有新样本时填 `gyro(rad/s)`、`accel(m/s²)`、`mag`，返回 true |
| `rc_read` | ✅ | 填 `roll/pitch/yaw ∈ [-1,1]`、`throttle ∈ [0,1]`、`mode_switch`、`valid` |
| `motor_write` | ✅ | 把 4 个 `[0,1]` 转成 DShot/PWM；**必须在此处做解锁/看门狗的硬件级互锁** |
| `time_us` | ✅ | 单调递增的 64 位微秒（用 32 位定时器 + 溢出中断扩展） |
| `baro_read` | ⬜ | `altitude_m`（相对起飞点，向上为正）、`climb_rate_mps` |
| `gnss_read` | ⬜ | 局部 NED 位置/速度（相对 home），`fix_type >= 3` 才用垂直速度 |
| `battery_read` | ⬜ | 经分压/滤波后的电压 |
| `log` | ⬜ | 已格式化好的字符串，可转 ITM/SWO/UART |
| `telemetry_write` | ⬜ | 一帧完整字节流，交给 UART DMA / USB CDC |
| `param_load` / `param_save` | ⬜ | Flash 最后一页 / EEPROM / FRAM |
| `watchdog_feed` | ⬜ | 喂独立看门狗 IWDG |

## 2. 传感器方向对齐（最容易出错的一步）

`imu_read` 必须把芯片原始轴对齐到 **机体 x 前 / y 右 / z 下**：

```c
/* 示例：BMI270 安装时旋转了 180° */
out->gyro  = (fcs_vec3_t){ raw_gx, -raw_gy, -raw_gz };
out->accel = (fcs_vec3_t){ raw_ax, -raw_ay, -raw_az };
```

检查方法：水平静止时 `accel ≈ (0, 0, -9.81)`；机头上仰时 `accel.x < 0`；
绕 z 轴顺时针（俯视）转机体时 `gyro.z > 0`。加速度计测量的是**比力**
（静止时读数指向"上"，即机体 −z），不是重力矢量。

单位：陀螺必须转成 rad/s，加速度计转成 m/s²，磁力计任意单位（内部会归一化）。
磁力计上机前必须做硬铁/软铁校准；未校准时把 `est_kmag` 设为 0。

## 3. 主循环挂接

推荐用 TIM 更新中断（1 kHz）或高优先级 RTOS 任务，不要放在 `while(1)` 里裸跑：

```c
static volatile uint64_t g_time_us;   /* 由 1 MHz 定时器 / DWT->CYCCNT 扩展 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM_FC) return;
    uint64_t now = bsp_time_us();
    if (now - g_last_tick >= 1000u) {
        g_last_tick = now;
        fcs_step(&g_fcs, now);
    }
}
```

要点：

- `fcs_step()` 内部不阻塞。UART/日志/遥测都走 DMA 或环形缓冲 + 中断，
  禁止在控制周期里 `HAL_UART_Transmit()` 轮询。
- 把 IMU DRDY 中断优先级设为**最高**，控制循环次之，遥测最低。
- `time_us()` 用 `DWT->CYCCNT` 或 TIM2 32 位计数器扩展，避免 `HAL_GetTick()` 只有 1 ms 分辨率。
- 独立看门狗超时设 20–50 ms；`watchdog_feed` 只在 `fcs_step()` 末尾被调用，
  所以主循环一旦卡死会自动复位（复位后状态机回到 DISARMED）。

## 4. 电机输出转换

```c
static void motor_write(const float *out, uint8_t count, void *ctx)
{
    for (uint8_t i = 0; i < count; ++i) {
        float v = out[i];
        if (!(v > 0.0f)) { bsp_dshot_send(i, 0); continue; }       /* 含 NaN 保护 */
        uint16_t pwm = (uint16_t)(MIN_PULSE_US + v * (MAX_PULSE_US - MIN_PULSE_US));
        bsp_dshot_send(i, pwm);   /* 或 __HAL_TIM_SET_COMPARE(&htim, ch, pwm) */
    }
}
```

- 归一化 `[0,1]` 映射到 DShot 油门 48–2047，或 PWM 1000–2000 µs。
- **上电时先输出 0 持续 2 s** 完成 ESC 校准，再允许解锁。
- 强烈建议在 BSP 层再加一个 GPIO 硬件急停（独立于软件），由地面站/物理开关控制。

## 5. 参数持久化

`fcs_params_t` 是 POD 结构体，直接整块写入 Flash 最后一页即可：

```c
static bool param_load(void *buf, uint16_t size, void *ctx)
{
    if (size > FLASH_PAGE_PARAM_SIZE) return false;
    memcpy(buf, (const void *)FLASH_PARAM_ADDR, size);
    return true;
}
```

写入前擦除扇区，并做 CRC/双备份（A/B 页）避免掉电写坏。即使存储损坏，
`fcs_params_load()` 会回落到 `fcs_params_defaults()`，不会带着随机增益起飞。

## 6. 上机检查清单

- [ ] 卸桨，接限流电源（建议 1 A 限流）
- [ ] 水平静止时 `accel ≈ (0,0,-9.81)`，`euler.roll/pitch < 1°`
- [ ] 用手倾斜机体，遥测中的 roll/pitch 方向与姿态一致
- [ ] 顺时针（俯视）转机体时 `yaw` 增大，`gyro.z > 0`
- [ ] 解锁互锁生效：油门不为最低时不解锁；解锁后电机最小油门
- [ ] 人为拔掉接收机 → 500 ms 内触发 `RC_LOSS` → LANDING/DISARMED
- [ ] 把机体翻转 60° → 电机立即停转
- [ ] `motor_write` 的 4 路电机编号与 `mixer.c` 矩阵完全一致（逐个轻推验证）
- [ ] 低电量、GNSS 丢失、IMU 断线分别做故障注入
- [ ] 首次实飞用 `STABILIZE`，确认手感后再开 `ALT_HOLD` → `POS_HOLD` → `AUTO`

## 7. 资源估算（Cortex-M4F, -O2）

| 项 | 估计 |
| --- | --- |
| `fcs_core` 代码 | ~15–20 KB |
| 静态 RAM（`fcs_t`） | ~1–2 KB |
| 单周期计算 | 约 3–8 µs @168 MHz（远小于 1 ms 周期） |
| 栈 | < 512 B（日志缓冲 192 B 在栈上） |

若目标 MCU 无 FPU（M0/M3），把 `float` 运算改为定点或换 M4F/M7，
否则 1 kHz 下软浮点可能吃满 CPU。
