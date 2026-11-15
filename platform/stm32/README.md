# STM32 移植模板

`hal_stm32_template.c` 是一份**不参与主机构建**的板级支持包骨架：

- 用 `extern` 声明 13 个 `bsp_*` 钩子，不依赖任何 Cube/HAL 头文件，因此可以直接阅读、拷贝；
- `fcs_stm32_hal()` 组装出完整的 `fcs_hal_t`；
- 文件末尾用 `#if 0` 包了一段 `main.c` 风格的应用骨架（初始化 + 1 kHz 定时中断 + 任务下发）。

按下列顺序完成移植：

1. 把 `include/` 与 `src/` 加入固件工程（或直接链接 `libfcs_core.a`）；
2. 实现 `bsp_*` 钩子：SPI/DMA 读 IMU、I²C 读气压计、UART 解析 GNSS/RC、ADC 电池、
   DShot/PWM 输出、DWT 微秒时钟、Flash 参数区、IWDG；
3. 对照 [docs/PORTING.md](../../docs/PORTING.md) 的**传感器方向对齐**与**上机检查清单**逐项验证；
4. 先卸桨跑 SIL 之外的硬件在环（把 `bsp_imu_read` 临时接到回放数据），再上电试车。

> 模板里 `FLASH_PARAM_ADDR = 0x080E0000` 只是占位，务必按具体芯片的扇区表修改，
> 并实现 A/B 双备份 + CRC，避免掉电写坏参数区。

CMake/Makefile 故意不包含本目录：它引用了尚未实现的 `bsp_*` 符号，
只链接 `fcs_core` 时不会有任何问题，但直接把整个目录加进主机构建会链接失败。
