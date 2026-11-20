# STM32H743 + BMI088：1000 Hz 六轴采集与姿态解算

独立的裸机 C 工程，默认从 **Flash `0x08000000`** 启动，用 ST-Link/SWD 直接烧录，不需要飞控固件或 bootloader。芯片、晶振、SPI、USB、电源引脚取自 [原始板级配置](docs/original-hwdef.dat)。

**当前版本每秒输出 1000 帧，每帧包含原始六轴、共同时间戳、四元数及 roll/pitch/yaw。** STM32H743 在主循环运行六轴 Mahony 姿态融合；USB 未连接时也持续更新，重连不重置姿态。BMM150 仅独立诊断采集、不参与姿态融合，yaw 是静置校准归零、允许漂移的相对航向，不是磁航向或真北。加热器始终关闭。

> 融合安装旋转沿用历史 `ROTATION_PITCH_180`（X/Z 取反），原始六轴不变；历史配置不等于实板方向已核验，完整实板验收仍未完成。历史 `release/`、`docs/validation.json` 和 `release/BUILD_INFO.json` 不代表本次姿态功能的交付或验收结果。

| 项目 | 设置 |
|---|---|
| MCU / 晶振 | STM32H743，2 MB Flash，16 MHz HSE 晶体，LDO |
| 时钟 | CPU 400 MHz，AHB 200 MHz，APB 100 MHz |
| USB 输出 | **固定 1000 Hz 六轴/姿态组合帧**，76 字节/帧，76000 B/s |
| 输出时间戳 | 同一个 TIM2 微秒时基上的绝对网格，`t0+n×1000 μs` |
| 加速度内部采样 | 1600 Hz，±24 g，Normal 滤波，XY 280 / Z 245 Hz 带宽 |
| 陀螺仪内部采样 | 2000 Hz，±2000 °/s，230 Hz 带宽 |
| SPI | SPI3，Mode 3，6.25 MHz，DRDY 中断读取 |
| USB | PA11/PA12，Full Speed CDC ACM 虚拟串口，打开并置 DTR 即输出 |

## BMM150 通信与磁场采集

新增 BMM150 独立采集：I²C4 PF14/PF15、7 位地址 `0x10`，核对 Chip ID `0x32`、读取工厂 trim 并补偿为 µT，正常模式约 25 Hz。磁场以独立 **72 字节 IMU2 v1/type 4** 帧与原有 76 字节 IMU 帧混合发送，不改六轴/姿态定义。磁场保留芯片轴，未应用安装旋转，**不做硬/软铁标定，也不纠正 yaw**。

操作、协议和验收边界见 **[BMM150 通信与磁场采集指南](docs/bmm150-validation.md)**。专用 `tools/mag_capture.py` 可采集和回放磁场 CSV，并同时统计原有 1 kHz IMU 链路；当前 GUI 只识别并跳过磁帧，不显示磁场。源代码和软件测试就绪不代表已经烧录或通过实板验收；实测文件仅存本地。

## 时间同步的范围

普通模式的 BMI088 加速度没有原生 1000 Hz 档位；两个部分也有各自的时钟。因此保留加速度 1600 Hz、陀螺仪 2000 Hz 原生读取，每个 1 ms 输出时刻分别选择**不晚于该时刻的最新有效原始样本**，组成一帧。高优先级 DRDY 中断负责读取，TIM2 比较中断仅选样、快照并送入有界队列；主循环按序融合、编码、计算 CRC 和处理 USB，不在 TIM2 中做浮点三角运算。原始六轴没有插值、零偏扣除或坐标旋转，也没有额外 SPI 重复读取；安装旋转和零偏仅用于融合输入。

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

生成 `build/imu_firmware.elf`、`.bin`、`.hex`、`.map`。HEX 自带地址；BIN 的烧录地址是 **`0x08000000`**。`release/` 为历史预编译文件，**尚未更新为本次 v3**，不可误用。完成测试及相应实板验收后可执行 `python3 scripts/package_release.py` 更新固件、校验和及完整工程 ZIP。

依赖版本与 SHA-256 见 [锁定文件](scripts/dependencies.lock.json)，许可证在 `third_party/`。依赖丢失时才需联网运行 `python3 scripts/bootstrap_deps.py --force`。

## 烧录

ST-Link 连接 **PA13=SWDIO、PA14=SWCLK、GND、3.3 V 电平参考**，建议连接 NRST。按板子原设计供电，不能默认 ST-Link 的 VTref 给整块板供电。STM32CubeProgrammer 选择 ST-LINK/SWD，下载本次重新构建的 `build/imu_firmware.hex`（或对应布局的 v3 构建目录产物），校验后复位。**不要把历史 release 文件当作新增姿态固件；烧录前核对链接地址与是否保留 bootloader。**

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

`capture.bin` 原样保存接收字节；`capture.csv` 每行是通过 CRC 校验的一帧六轴数据，含共同时间、两组源时间与年龄，以及姿态有效性、四元数和角度；无效或 v2 帧的姿态数值列留空。离线解码不需要 pyserial：

```bash
python3 tools/imu_capture.py --input capture.bin --stats
python3 tools/imu_capture.py --input capture.bin --csv units.csv --units --stats
```

`--units` 保留原始整数，额外添加 g 和 °/s 六列。主机工具兼容先前 IMU1、IMU2 v2 文件，新固件常规采样输出 IMU2 v3；显式手动重校准时另发送 32 字节控制 ACK，传感器 CSV 不增加控制行。

新协议为 **IMU2 v3/type 3，76 字节、小端序**：

| 偏移 | 字段 |
|---:|---|
| 0 | `IMU2` 四字节帧头 |
| 4 / 5 / 6 | version=3 / type=3 / uint16 length=76 |
| 8 | uint32 sequence |
| 12 | uint64 timestamp_us，共同输出参考时间 |
| 20 | int16 ax、ay、az |
| 26 | int16 gx、gy、gz |
| 32 / 36 | uint32 accel_age_us / gyro_age_us |
| 40 / 42 | uint16 flags / tick_late_us |
| 44 / 48 / 52 / 56 | float32 qw、qx、qy、qz（机体到世界） |
| 60 / 64 / 68 | float32 roll、pitch、yaw（度） |
| 72 | CRC32，`zlib.crc32(前72字节)` |

完整定义、质量标志、解析和重同步规则见 [协议文档](docs/protocol.md)。换算为 `a[g]=raw×24/32768`、`ω[°/s]=raw×2000/32768`。**原始六轴不应用安装旋转。** 融合输出为机体 FRD → 世界 NED 的标量在前四元数；Euler 为 3-2-1 顺序、度。flags 新增 bit 10 校准中、bit 11 姿态有效、bit 12 融合重置；无效帧的线上姿态只是占位，接收端不得显示或保存为有效测量。旧 v2 保留离线解析。

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

融合快照队列为 **64 项**，独立发送队列为 **512×76 字节**，TinyUSB 另有 4096 字节发送缓冲。主循环约每 1 ms 提交发送数据，USB 可批量交付。发送背压丢帧只报告发送丢失，不重置姿态；融合队列丢帧或数据中断超过 10 ms 则标记无效并重新校准。短暂漏节拍不补造样本，仅积分当前至多 1 ms；过期、错误、重复或回退的陀螺源样本不积分。

USB 关闭、卸载或挂起只清空发送队列、重建发送会话与序号，不清空采样历史或姿态。重开后的姿态延续断开期间的运动；真实采样中断除外。USB 已提交的在途包可能含少量前会话数据，按新会话标志重建基线。

### 校准与融合默认参数

完整操作流程、校准原理、30 Hz 静置检测滤波用途及限制见 **[IMU 校准说明](docs/imu-calibration.md)**。

上位机新增 **重新校准 IMU** 按钮，确认后无需重启即可重新估计零偏和重力倾角，并将相对 yaw 重新归零；原始采样、时间戳、序号和录制继续。执行确认 ACK 不等于校准完成，需等待重置边界之后的有效姿态；超时结果未知且不自动重试。操作步骤见 [手动重新校准](docs/imu-calibration.md#21-从上位机手动重新校准)，命令及 ACK 布局见 [协议文档](docs/protocol.md)。需配套更新固件和上位机，不能假定已烧录的旧固件支持。

200 ms 窗口填满后，连续约 2 秒满足静置判据才估计陀螺零偏、用重力初始化倾角并将 yaw 归零（稳定上电通常约 **2.2 秒**）。移动、坏样本或不连续节拍会重新累计静置时间：

- 当前角速度模长 <5 °/s，加速度模长距 1 g ≤0.1 g。
- 200 ms 窗口内原始加速度各轴标准差 ≤0.02 g，30 Hz 检测专用低通后的角速度各轴标准差 ≤0.5 °/s；该低通仅用于静置判断，不改变原始输出、零偏均值或姿态积分。
- Mahony `Kp=2 s⁻¹`、`Ki=0`；加速度模长或短时波动不适合重力修正时，仅积分陀螺。

这些是首版固定参数，阈值可靠性、实板安装方向及运行开销尚需验收。安装旋转只在 `firmware/include/fusion.h` 独立配置，不能改动原始轴定义来迁就姿态。

运行测试：

```bash
python3 -m unittest discover -s tests -v
```

自动化测试覆盖真实 C 编码器/选样/融合/缓冲、CRC 参考向量、静置与运动校准、倾斜与三轴转动、相对 yaw 漂移、过期/重复样本、漏节拍、融合/发送队列丢失、USB 断开重连；以及 v2/v3 拆包重同步、无效姿态屏蔽、有界趋势、录制重解校验和导出兼容。C 测试需要主机编译器，没有编译器时该组会跳过；是否启用 UBSan 取决于运行平台与工具链。上位机测试命令见 [上位机说明](viewer/README.md)。板端状态可通过 SWD 查看 `g_imu_diagnostics` 与 `g_last_imu_frame`；运行中读取后一项应检查 CRC 并重试，调试暂停本身会制造漏节拍。

软件测试不能替代实板定时、轴向、姿态精度和 **76 kB/s** USB 连续传输验收；`docs/validation.json` 和 `release/BUILD_INFO.json` 仅为此前版本记录。采集数据、烧录日志及本轮验证报告仅在本地保留，不随功能提交。

## 源码

- `firmware/src/board.c`：时钟、电源、SPI、微秒时基、定时合帧调度、EXTI。
- `firmware/src/bmi088.c`：初始化、配置回读和原始数据读取。
- `firmware/src/board_i2c.c` / `mag.c`：非阻塞 I²C4、BMM150 配置/回读、工厂补偿及独立磁场诊断。
- `tools/mag_capture.py`：混合流磁场采集、CSV、统计与离线回放。
- `firmware/src/stream.c`：历史选样、快照队列、主循环融合调度、质量标志、发送队列、手动重校准命令及 USB 会话。
- `firmware/src/fusion.c`：静置校准、独立安装旋转、六轴 Mahony、四元数/Euler 与失效重置。
- `firmware/src/protocol.c`：二进制帧及 CRC。
- `tools/imu_capture.py`：主机采集、v2/v3 解码、CSV、输出和对齐统计。
- `viewer/`：双目/IMU 实时预览、姿态与有界趋势、录制四元数详情、离线校验与 EuRoC 导出，见 [上位机说明](viewer/README.md)。

原生速率旧版工程归档保存在本地 `history/Flight_controller_2_IMU_native_rates_v1.zip`，不包含在当前 Git 仓库或工程交付 ZIP 内。保留 bootloader 的 128 KB 偏移构建仍可选：`./scripts/build.sh build-bootloader -DFLASH_OFFSET_KB=128`；默认交付从零启动。
