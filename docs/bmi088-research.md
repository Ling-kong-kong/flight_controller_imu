# BMI088 原始采样配置核对（1000 Hz 合并输出）

依据 Bosch BMI088 数据手册 Rev.1.9（2024-01）及 Bosch BMI08x SensorAPI；访问日期 2026-11-17。此文是寄存器核对记录，实际实现见 `firmware/src/bmi088.c`。

## 默认寄存器

| 部分 | 地址 | 写入值 | 含义 |
|---|---:|---:|---|
| Accel | `0x7C` ACC_PWR_CONF | `0x00` | Active 配置 |
| Accel | `0x7D` ACC_PWR_CTRL | `0x04` | 开启加速度计 |
| Accel | `0x40` ACC_CONF | `0xAC` | 原生 1600 Hz，Normal 滤波 |
| Accel | `0x41` ACC_RANGE | `0x03` | ±24 g |
| Accel | `0x53` INT1_IO_CONF | `0x0A` | INT1 输出使能，高有效、推挽 |
| Accel | `0x58` INT1_INT2_MAP_DATA | `0x04` | DRDY 映射到 INT1 |
| Gyro | `0x11` GYRO_LPM1 | `0x00` | Normal 模式；上电和软复位后默认如此 |
| Gyro | `0x0F` GYRO_RANGE | `0x00` | ±2000 °/s |
| Gyro | `0x10` GYRO_BANDWIDTH | `0x01` | 原生 2000 Hz，230 Hz 带宽，适合 1000 Hz 输出 |
| Gyro | `0x16` INT3_INT4_IO_CONF | `0x01` | INT3 高有效、推挽；INT4 未映射 |
| Gyro | `0x18` INT3_INT4_IO_MAP | `0x01` | DRDY 映射到 INT3 |
| Gyro | `0x15` GYRO_INT_CTRL | `0x80` | 使能 DRDY |

加速度 Normal、1600 Hz 时，3 dB 带宽为 X/Y 280 Hz、Z 245 Hz。`GYRO_BANDWIDTH` 的 bit 7 只读且恒为 1，无功能；回读比较必须屏蔽该位。加速度原始 XYZ 从 `0x12` 连读 6 字节，陀螺仪从 `0x02` 连读 6 字节，均为低字节在前的有符号 16 位数据。

## SPI 和启动时序

- SPI Mode 0 或 Mode 3；最高 10 MHz。工程使用 Mode 3、6.25 MHz。CS 建立、保持时间至少各 40 ns；工程各留 1 μs。
- 两个部分有独立片选；配置文件明确 `PI4=Accel CS`、`PI8=Gyro CS`。`PI6=Accel INT1`、`PI7=Gyro INT3` 是基于 DRDY 命名的接线假设，配置文本本身没有芯片引脚级连线，仍须用原理图或示波器确认。
- 加速度计上电首先为 I²C 模式，需要一次 CS 上升沿切换 SPI。做一次忽略结果的 CHIP_ID 读取，之后再正式读取 `0x1E`；软复位之后也重新执行。
- 每次加速度 SPI 读取都是 `地址|0x80`、一个 dummy 字节、有效数据。陀螺仪没有这个 dummy 字节，其 CHIP_ID 为 `0x0F`。
- Accel 向 `0x7E` 写 `0xB6` 软复位，等待至少 1 ms。Gyro 向 `0x14` 写 `0xB6`，等待至少 30 ms，期间不要通信。Gyro 电源模式切换也需要最多 30 ms。
- 加速度复位后处于 suspend，需要主动开启。Bosch API 的保守顺序为 `PWR_CONF=0`、等 5 ms、`PWR_CTRL=4`、等 5 ms。普通写后，Normal 模式至少等 2 μs；Suspend 模式采用至少 1 ms 的保守等待。
- Gyro DRDY 在每次完整 XYZ 更新后触发，并在 280–400 μs 后自动清除。使用高有效上升沿 EXTI 可以处理 500 μs 的原生采样周期。

## 为什么内部保持两个原生速率，输出统一 1000 Hz

普通模式的加速度计最高 1600 Hz，没有原生 1000 Hz 档位；陀螺仪最高 2000 Hz，内部时钟独立。当前保持 1600/2000 Hz 原始采集，由同一个 MCU 微秒定时器每 1000 μs 选择不晚于输出时刻的最新有效原始样本，组成完整六轴帧。帧内保留两部分源时间的年龄，可以恢复 DRDY 服务时间差；不插值，也不宣称物理转换同时发生。230 Hz 陀螺仪带宽与加速度 280/245 Hz 带宽均低于 1000 Hz 输出的奈奎斯特频率，但有限阻带衰减仍不等于理想抗混叠滤波。

Bosch 另外提供 400/1000/2000 Hz 的同步功能，其中 2000 Hz 加速度结果由原生 1600 Hz 数据进行内部插值得到。该模式需要加载 feature configuration，并将陀螺仪 DRDY **物理连接**至可配置为输入的加速度中断脚。用户提供的 hwdef 不能证明存在该连接；本工程采用两个 DRDY 分别读取原始寄存器，不启用同步插值功能。

## 来源

- [BMI088 官方数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)：章节 3、4.1、4.4、4.7、5.3、5.5、6.1。
- [Bosch BMI08x SensorAPI](https://github.com/boschsensortec/BMI08x_SensorAPI/tree/c1ed227e7bb7da1fa600bbd4e5c82d0da1eb416a)：核对提交 `c1ed227e7bb7da1fa600bbd4e5c82d0da1eb416a` 的 `bmi08_defs.h`、`bmi08a.c`、`bmi08g.c`。
- [Bosch 数据同步说明](https://github.com/boschsensortec/BMI08x_SensorAPI/blob/master/DataSync.md)：同步所需连线、插值原理和配置要求。
