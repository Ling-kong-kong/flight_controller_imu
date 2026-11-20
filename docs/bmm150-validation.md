# BMM150 通信与磁场采集指南

## 范围与边界

本阶段只验证通信、配置、工厂补偿及独立磁场采集。**不实现九轴融合，不改变 yaw，不做硬铁/软铁标定。** 原有 BMI088 1 kHz 六轴、FRD/NED 姿态、安装旋转、手动重新校准均保留。BMM150 输出采用芯片寄存器轴，尚未旋转到 IMU 机体轴；历史 `ROTATION_YAW_180` 不能替代实板核验。

本文是产品操作说明，不是实板验收记录。采集、烧录日志、实测报告放入已忽略的 `logs/bmm150/`，不要提交；历史 release 不代表本功能已烧录。

## 板级接口与实现

- 原理图 U14 为 BMM150：PS/CSB 接高、SDO 接地，对应 I²C 7 位地址 **0x10**；SCK/SDI 接 MCU **PF14 SCL / PF15 SDA，AF4 I²C4**。
- VDD/VDDIO 来自传感器 3.3 V；无可用 DRDY 接线，采用轮询。必须有合适的外部上拉，固件不启用内部上拉；上拉和边沿需实测。
- I²C4 内核 100 MHz，TIMINGR `0x90923131`，目标不超过 100 kHz。事务只在主循环协作执行，每次最多 16 字节，5 ms 超时；DRDY/TIM2 中不做 I²C。
- 总线还连接其他器件。只访问 0x10，不扫描、不全总线复位、不输出 GPIO 时钟恢复脉冲。错误只恢复本控制器并限速重试；若从机持续拉低总线，可能需要检查接线/断电恢复。

初始化：`4B=01` 唤醒、等待 3 ms → `40` 读 ID `32` → 读取 `5D..5E / 62..65 / 68..71` 工厂 trim → `4C=36` sleep → `51=04 / 52=07` repetition → `4E=00` 启用三轴 → `4C=30` normal/25 Hz → 回读 `4B..52`。trim 全零/全 FF 或关键分母参数为零会拒绝。

每次数据读取完成后等待 10 ms，先单独读取 `48` bit0 检查 ready。确认 ready 后立即连续读取 `42..49`，得到同一快照的 XYZ/RHALL。单独读取 ready 会在 STOP 时清除状态，因此数据 burst 不再要求该位为 1，线上 ready 表示前一步已确认新测量。不能先启动未 ready 的整段读取再看末尾 ready：转换若发生在 burst 期间，可能将旧的 shadow 快照误判为新样本。不得按数值去重，真实新样本也可能数值相同。无 ready 只累计计数，不造样本；200 ms 无 ready 判故障。ID、总线、trim、回读故障发送诊断并以 1 秒间隔重试，不使 IMU 致命停机。

使用 Bosch BMM150 SensorAPI v2.0.0 浮点补偿（BSD-3-Clause），只应用出厂 trim，不等于环境磁标定。溢出哨兵、零 RHALL、异常分母/非有限输出均无效。USB 断开仍采集；磁发送只有一个有界 slot，背压覆盖旧记录并计数。IMU 发送优先，磁帧可能被丢弃，不承诺无背压时以外的可靠交付。

## 采集与离线回放

先编译本次固件并按主 README 的 ST-Link 步骤下载、校验、复位；不要误烧历史 release。确认上位机已断开串口，避免抢占。

```powershell
New-Item -ItemType Directory -Force logs/bmm150
python tools/mag_capture.py --port COM12 --seconds 30 --raw logs/bmm150/usb.bin --csv logs/bmm150/mag.csv --summary logs/bmm150/summary.json
python tools/mag_capture.py --input logs/bmm150/usb.bin --csv logs/bmm150/replay.csv --summary logs/bmm150/replay.json
```

替换为实际端口；实时采集需要 `tools/requirements.txt` 的 pyserial，回放不需要。工具只打开 DTR，**不向板端发送控制命令**。同一时刻只用一个串口读者。

`usb.bin` 保留原始混合流；`mag.csv` 包含原始 XYZ、RHALL、µT、模长、有效性及诊断。无效帧 µT/模长留空，原始值和故障字段保留。summary 同时统计磁场和原有 IMU 健康，内存有界。实时和回放 CSV 应逐字节一致；同一原始流的统计结果也应一致。

现有 `imu_capture.py` 解析并统计磁帧，但不把它们放入六轴 CSV；GUI 验证磁帧 CRC 后跳过，原始 `usb.bin` 可包含磁帧。GUI 没有新增磁场视图。EuRoC 六轴文件不增加磁场列。

## 磁场帧协议

`IMU2 v1/type 4`，**72 字节，小端**，与 76 字节 IMU v3、旧 v2 及控制 ACK 混合。解析器应根据 version/type/length 分帧并用 CRC 重同步，不能按固定 76 字节切流。

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | char[4] | `IMU2` |
| 4 / 5 / 6 | u8 / u8 / u16 | version=1 / type=4 / length=72 |
| 8 | u32 | sequence，样本和诊断共用，USB 重连不归零 |
| 12 | u64 | timestamp_us，MCU 寄存器读完成时间，**不是转换/DRDY 时间** |
| 20 / 22 / 24 | 3×i16 | 芯片轴 raw X/Y/Z，未旋转 |
| 26 | u16 | RHALL |
| 28 / 32 / 36 | 3×float32 | 工厂 trim 补偿 X/Y/Z，µT |
| 40 | u16 | flags |
| 42 / 43 | u8 / u8 | address / chip_id |
| 44 / 45 | u8 / u8 | state / error |
| 46 / 47 | u8 / u8 | repXY=4 / repZ=7 |
| 48 / 52 | u32 / u32 | 累计 bus_errors / invalid_samples |
| 56 / 60 | u32 / u32 | 累计 not_ready / tx_overwrites |
| 64 | u32 | read_us，8 字节数据 burst 耗时（不含前置 ready 检查） |
| 68 | u32 | CRC32=`zlib.crc32(前68字节)` |

flags：bit0 valid、bit1 ready、bit2 trimOK、bit3 configured、bit4 diagnostic。当前有效样本 flags=15，diagnostic 不作为测量。接收端还检查地址/ID、state=6、error=0、repetition、raw 范围、RHALL 1..16383、有限 µT。无效线上数值只作占位，不显示为有效测量。

state：0 错误/等待重试，1 唤醒，2 ID，3 trim，4 配置，5 回读，6 采集。error：0 正常，1 总线，2 ID，3 trim，4 配置回读，5 ready 超时，6 补偿无效。`not_ready` 是最多约 100 Hz 的 ready 轮询无新数据次数，25 Hz 器件下正常增长，**不是丢帧数**。`tx_overwrites` 表示旧待发磁帧被新帧覆盖，本次覆盖会在后续帧计数中体现。USB 关闭时不积压磁帧，也不因此增加覆盖计数。

## 实板验收顺序

1. 静置、远离磁铁/扬声器/钢桌和大电流线，采集 30 秒。确认 `chip_id=50(0x32)`、state=6、error=0、trim/config 标志；有效率约 25 Hz，RHALL 非零，数据有限，无总线错误或补偿错误增长。
2. 同时检查 IMU 约 1000 Hz，CRC/序号缺口/旧样本/SPI/融合队列异常不增长；磁场不能破坏原链路。正常总带宽约 **76000+25×72=77800 B/s**，另有少量诊断/ACK。
3. 离线重解同一 usb.bin，比较磁 CSV/统计。关闭再重开 DTR，确认重新收流且不重置现有姿态；磁序号可因断线出现缺口。
4. 用户缓慢绕三轴转动，观察磁分量连续、方向变化合理，确认轴映射。不要用“静置成功”代替转动验收。转动时模长若随方向明显变化，可能存在硬/软铁影响，需后续标定，不能直接把磁场喂给 yaw。
5. 典型地磁模长约 25–65 µT，只用于合理性筛查，不是本地绝对阈值或标定依据。大偏差应排查局部磁场、trim、供电和安装。没有仪器/参考航向不能证明绝对磁精度。

SWD 可只读查看 `g_mag_diagnostics`、`g_mag_trim`。暂停调试会影响时序，不把调试制造的漏节拍当作正常运行结果。软件 native/解析回归不能证明物理连线、上拉、电气边沿、实际轴向或磁精度已通过。
