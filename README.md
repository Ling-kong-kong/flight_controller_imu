# STM32H743 + BMI088：1000 Hz 六轴原始数据采集板

独立的裸机 C 工程，默认从 **Flash `0x08000000`** 启动，用 ST-Link/SWD 直接烧录，不需要飞控固件或 bootloader。芯片、晶振、SPI、USB、电源引脚取自 [原始板级配置](docs/original-hwdef.dat)。

**当前版本每秒输出 1000 帧，每帧同时包含三轴加速度、三轴角速度和一个共同时间戳。** USB 只传原始轴整数及封包、时间对齐和质量判断所需的元数据，不传罗盘、气压、温度、姿态或调试文本。加热器始终关闭。

| 项目 | 设置 |
|---|---|
| MCU / 晶振 | STM32H743，2 MB Flash，16 MHz HSE 晶体，LDO |
| 时钟 | CPU 400 MHz，AHB 200 MHz，APB 100 MHz |
| USB 输出 | **固定 1000 Hz 六轴合并帧**，48 字节/帧，48000 B/s |
| 输出时间戳 | 同一个 TIM2 微秒时基上的绝对网格，`t0+n×1000 μs` |
| 加速度内部采样 | 1600 Hz，±24 g，Normal 滤波，XY 280 / Z 245 Hz 带宽 |
| 陀螺仪内部采样 | 2000 Hz，±2000 °/s，230 Hz 带宽 |
| SPI | SPI3，Mode 3，6.25 MHz，DRDY 中断读取 |
| USB | PA11/PA12，Full Speed CDC ACM 虚拟串口，打开并置 DTR 即输出 |

## 时间同步的范围

普通模式的 BMI088 加速度没有原生 1000 Hz 档位；两个部分也有各自的时钟。因此保留加速度 1600 Hz、陀螺仪 2000 Hz 原生读取，每个 1 ms 输出时刻分别选择**不晚于该时刻的最新有效原始样本**，组成一帧。高优先级 DRDY 中断负责读取，TIM2 比较中断负责合帧，没有软件插值、零偏扣除、坐标旋转、姿态解算或额外 SPI 重复读取。

共同时间戳表示输出参考时刻；帧内还保留 `accel_age_us`、`gyro_age_us`，可以恢复两个部分各自的 DRDY 服务时间：

```text
加速度源时间 = 共同时间戳 - accel_age_us
陀螺仪源时间 = 共同时间戳 - gyro_age_us
两组数据的时间差 = abs(accel_age_us - gyro_age_us)
```

健康采集时，年龄通常分别在约一个 625 μs 和 500 μs 的原生周期以内，包含中断服务时序的影响。这统一了输出节拍、封包和时间基准，**不代表两部分同时完成物理转换**。原生 DRDY 时间包含响应延迟，器件内部滤波的群延迟也可能不同。严格硬件同步需要额外的陀螺仪 DRDY → 加速度中断输入连线和 Bosch feature 配置；提供的 hwdef 没有证明这条连接存在，本工程未开启该功能。[寄存器与官方来源核对](docs/bmi088-research.md)提供了依据。

如果定时处理中断迟到超过整毫秒，跳过错过的节拍并反映为序号缺口，不补造旧样本。历史中找不到合适样本时跳过该帧，不输出全零占位数据。SPI 暂时失败时可继续使用较旧的有效数据，但会报告错误和样本年龄；年龄超过 1000 μs 会置旧数据标志。

## 编译

需要 ARM GNU Toolchain（`arm-none-eabi-gcc`）、CMake ≥3.20、Make 或 Ninja、Python ≥3.10。已包含所需 HAL、CMSIS、TinyUSB 源码，正常编译无需联网：

```bash
./scripts/build.sh
```

或者：

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

生成 `build/imu_firmware.elf`、`.bin`、`.hex`、`.map`。HEX 自带地址；BIN 的烧录地址是 **`0x08000000`**。预编译文件在 `release/`。测试并构建后可执行 `python3 scripts/package_release.py` 更新固件、校验和及完整工程 ZIP。

依赖版本与 SHA-256 见 [锁定文件](scripts/dependencies.lock.json)，许可证在 `third_party/`。依赖丢失时才需联网运行 `python3 scripts/bootstrap_deps.py --force`。

## 烧录

ST-Link 连接 **PA13=SWDIO、PA14=SWCLK、GND、3.3 V 电平参考**，建议连接 NRST。按板子原设计供电，不能默认 ST-Link 的 VTref 给整块板供电。STM32CubeProgrammer 选择 ST-LINK/SWD，下载 `release/imu_firmware.hex`，校验后复位。

OpenOCD 示例：

```bash
openocd -f interface/stlink.cfg -c "transport select swd" \
  -f target/stm32h7x.cfg -c "adapter speed 100" \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -c "adapter srst delay 100" \
  -c "program build/imu_firmware.elf verify reset exit"
```

上述复位连接方式需要 NRST 已接。未接 NRST 时应使用对应的普通 SWD/软件复位连接方式。默认布局会替换 Flash 起始位置的旧固件或 bootloader；上电启动脚应设为从用户 Flash 启动。

## USB 接收与协议

板载 USB 连到电脑后，Linux 一般出现 `/dev/ttyACM0`，Windows 10/11 出现 COM 端口。ST-Link 的 USB 接口仅用于烧录调试。安装 pyserial 并采集：

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/imu_capture.py --port /dev/ttyACM0 --seconds 10 \
  --raw capture.bin --csv capture.csv --stats
```

Windows 将端口改为实际的 `COM5` 等。程序会打开 DTR，无需发送命令。115200 等虚拟串口波特率不限制 USB 带宽。普通文本串口工具显示乱码属于二进制数据的正常表现。

`capture.bin` 原样保存接收字节；`capture.csv` 每行是通过 CRC 校验的一帧六轴数据，含共同时间、两组源时间与年龄。离线解码不需要 pyserial：

```bash
python3 tools/imu_capture.py --input capture.bin --stats
python3 tools/imu_capture.py --input capture.bin --csv units.csv --units --stats
```

`--units` 保留原始整数，额外添加 g 和 °/s 六列。主机工具兼容先前 IMU1 文件，新固件仅发送 IMU2。

新协议为 **48 字节、小端序**：

| 偏移 | 字段 |
|---:|---|
| 0 | `IMU2` 四字节帧头 |
| 4 / 5 / 6 | version=2 / type=3 / uint16 length=48 |
| 8 | uint32 sequence |
| 12 | uint64 timestamp_us，共同输出参考时间 |
| 20 | int16 ax、ay、az |
| 26 | int16 gx、gy、gz |
| 32 / 36 | uint32 accel_age_us / gyro_age_us |
| 40 / 42 | uint16 flags / tick_late_us |
| 44 | CRC32，`zlib.crc32(前44字节)` |

完整定义、质量标志、解析和重同步规则见 [协议文档](docs/protocol.md)。换算为 `a[g]=raw×24/32768`、`ω[°/s]=raw×2000/32768`。没有应用原飞控的 `ROTATION_PITCH_180`。

## 引脚和首次上板检查

| 功能 | STM32 引脚 | 说明 |
|---|---|---|
| SPI3 SCK / MISO | PC10 / PC11，AF6 | BMI088 共用总线 |
| SPI3 MOSI | PB2，**AF7** | 与 PC10/PC11 的 AF 不同 |
| 加速度 / 陀螺仪 CS | PI4 / PI8 | 低有效，独立片选 |
| 加速度 DRDY | PI6 | 按 BMI088 INT1 实现 |
| 陀螺仪 DRDY | PI7 | 按 BMI088 INT3 实现 |
| USB D− / D+ | PA11 / PA12 | OTG_FS 内置 PHY |
| USB VBUS | PA9 | 输入，不驱动 |
| 传感器电源使能 | PI11、PF4、PE7、PG8 | 按原配置输出高电平 |
| 加热器 | PB10 | 始终输出低电平 |
| RGB LED | PE3 / PE4 / PE5 | 红 / 绿 / 蓝，开漏低电平亮 |

hwdef 只给出 PI6/PI7 的 DRDY1/2 名称，未明确 BMI088 芯片侧的 INT 编号。当前采用 INT1/INT3；如果实板接 INT2/INT4，应调整 `bmi088.c` 的中断配置与映射，PI6/PI7 对调则调整 `board.h`。之前原生速率固件已通过 SWD 验证两路 DRDY 工作、SPI 错误为 0；新版的实板验证结果单独记录，不能代替 USB 连续接收测试。

启动会检查 Chip ID 并回读配置。蓝灯每 0.5 秒翻转表示主循环工作；打开串口时绿灯亮。任意一次运行中的 SPI 失败使红灯保持亮。致命故障则红灯按以下次数成组闪烁，组间隔约 1 秒：

| 次数 | 含义 |
|---:|---|
| 1 / 2 | 电源或时钟配置失败 / SPI3 初始化失败 |
| 3 / 4 | 加速度 ID 不是 0x1E / 陀螺仪 ID 不是 0x0F |
| 5 / 6 | 复位后识别或配置回读失败 / Cortex-M 异常 |
| 7 | 任一 DRDY 连续约 100 ms 无中断 |
| 8 / 9 | USB 整帧写入内部约束失败 / USB 栈初始化失败 |

采集 10 秒，再采集 60 秒检查：`imu.received_hz` 约 1000、连续共同时间戳间隔 1000 μs、无序号缺口和 CRC 错误、SPI/队列/旧数据/漏节拍标志为 0，会话首帧标志是正常的。**固定网格的周期标准差为零，不表示原生采样抖动为零**；还应查看 `tick_late_us`、两组年龄和 `source_skew_us`。USB 到达间隔会受主机调度和批量传输影响。

## 时序、缓冲和验证

中断优先级为 DRDY EXTI=1、TIM2=2、USB=3、SysTick=15。TIM2 的 32 位溢出标志清除与高位递增原子执行；同一时基既记录 DRDY，又安排每 1000 μs 的比较中断。每组保留 8 个历史样本，合帧只选过去的数据。SPI 采用 DWT 周期计数实现 200 μs 超时，避免中断内依赖低优先级 SysTick。正常上电仅启用 I-cache。

合并帧队列为 512×48 字节，TinyUSB 另有 4096 字节发送缓冲。主循环约每 1 ms 提交队列数据，USB 可批量交付；长时间不读取仍会溢出，并通过序号和 flags 报告。关闭串口清空软件输出队列，但保留源采样历史，重开立即输出真实样本。USB 已提交的在途包可能含少量前会话数据，按新会话标志重建基线。

运行测试：

```bash
python3 -m unittest discover -s tests -v
```

**38 项测试通过**：20 项主机协议测试，18 项真实 C 编码器/选样/缓冲集成测试，包含黄金向量、未来样本拒绝、源时间差、定时漏拍、32 位定时器回绕时间、SPI 失败、队列溢出、USB 背压和重连。C 测试需要主机 `cc`，使用 UBSan；没有编译器时该组会跳过。板端状态可通过 SWD 查看 `g_imu_diagnostics` 与 `g_last_imu_frame`；运行中读取后一项应检查 CRC 并重试，调试暂停本身会制造漏节拍。

软件测试不能替代实板定时和 USB 链路验证。交付时的实际烧录与硬件验证状态见 `docs/validation.json` 和 `release/BUILD_INFO.json`。

## 源码

- `firmware/src/board.c`：时钟、电源、SPI、微秒时基、定时合帧调度、EXTI。
- `firmware/src/bmi088.c`：初始化、配置回读和原始数据读取。
- `firmware/src/stream.c`：历史选样、六轴合帧、质量标志、队列、USB 会话。
- `firmware/src/protocol.c`：二进制帧及 CRC。
- `tools/imu_capture.py`：主机采集、解码、CSV、输出和对齐统计。

原生速率旧版工程归档保存在本地 `history/Flight_controller_2_IMU_native_rates_v1.zip`，不包含在当前 Git 仓库或工程交付 ZIP 内。保留 bootloader 的 128 KB 偏移构建仍可选：`./scripts/build.sh build-bootloader -DFLASH_OFFSET_KB=128`；默认交付从零启动。
