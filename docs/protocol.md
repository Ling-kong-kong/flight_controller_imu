# IMU2 USB 六轴原始数据协议（版本 2，固定 1000 Hz）

USB 为 Full Speed CDC ACM 虚拟串口。打开串口并置 DTR 后持续接收二进制数据，无需发送命令。串口设置的 115200 等波特率只是 CDC 兼容参数，不限制 USB 实际带宽。数据流只包含 BMI088 原始六轴值及封包、时间和质量判断所需的元数据，不输出温度、罗盘、气压、姿态或调试文本。

**每秒输出 1000 帧，每帧同时包含加速度 XYZ、陀螺仪 XYZ 和一个共同的输出时间戳。** 每帧 48 字节，名义流量为 **48000 B/s**。USB 会合并多个样本交付，USB 包边界和串口一次 `read()` 的长度都不是本协议的帧边界。

## 时间对齐的含义

MCU 使用同一个 1 MHz 定时器计时和安排输出，共同时间戳为 `t0 + n × 1000 μs` 的固定网格。BMI088 内部仍使用加速度 **1600 Hz、±24 g**、陀螺仪 **2000 Hz、±2000 °/s** 原生采样；每个输出时刻分别选择时间戳**不晚于该时刻的最新原始样本**，把两部分放入同一帧。不做软件插值，也不把某个传感器的时间戳改写成另一个传感器的时间戳。

BMI088 的两部分内部时钟独立。当前接线未确认具备官方同步功能所需的陀螺仪 DRDY → 加速度中断输入连线，因此这里统一的是**输出节拍和时间基准**，不能宣称两部分同时完成物理转换。帧内保留两路源数据相对共同时间戳的年龄：

```text
accel_time_us = timestamp_us - accel_age_us
gyro_time_us  = timestamp_us - gyro_age_us
source_skew_us = abs(accel_age_us - gyro_age_us)
```

这两个源时间戳指的是相应 DRDY 被 MCU 服务的时刻，包含中断响应延迟，不是精确 ADC 转换时刻或 BMI088 内部 sensortime。正常情况下，加速度年龄约在一个 625 μs 原生周期以内，陀螺仪年龄约在一个 500 μs 周期以内，实际还受中断响应和 SPI 读取时间影响。两部分内部滤波的群延迟也不一定相同。

为了适合 1000 Hz 输出，加速度使用 Normal 内部滤波（XY 280 Hz、Z 245 Hz 带宽），陀螺仪使用 2000 Hz / 230 Hz 带宽设置。XYZ 保持芯片输出寄存器的原始整数，不做零偏扣除、比例标定、坐标旋转或姿态解算；原飞控配置中的 `ROTATION_PITCH_180` 不应用于数据。

## 每帧 48 字节

所有多字节字段为**小端序**，轴数据为二进制补码 `int16`。没有 ASCII 日志、行尾或转义。

| 偏移 | 长度 | 字段 | 定义 |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII `IMU2`，十六进制 `49 4D 55 32` |
| 4 | 1 | version | `uint8`，固定为 2 |
| 5 | 1 | type | `uint8`，固定为 3，六轴合并帧 |
| 6 | 2 | length | `uint16`，固定为 48 |
| 8 | 4 | sequence | `uint32`，每个输出节拍递增，按模 2³² 回绕 |
| 12 | 8 | timestamp_us | `uint64`，该输出节拍的共同参考时间 |
| 20 | 2 | ax_raw | 加速度 X，`int16` |
| 22 | 2 | ay_raw | 加速度 Y，`int16` |
| 24 | 2 | az_raw | 加速度 Z，`int16` |
| 26 | 2 | gx_raw | 陀螺仪 X，`int16` |
| 28 | 2 | gy_raw | 陀螺仪 Y，`int16` |
| 30 | 2 | gz_raw | 陀螺仪 Z，`int16` |
| 32 | 4 | accel_age_us | `uint32`，加速度 DRDY 服务时间距共同时间的差 |
| 36 | 4 | gyro_age_us | `uint32`，陀螺仪 DRDY 服务时间距共同时间的差 |
| 40 | 2 | flags | `uint16`，见下表 |
| 42 | 2 | tick_late_us | `uint16`，定时器中断服务时刻距该输出节拍的延迟，超过 65535 时饱和 |
| 44 | 4 | crc32 | 前 44 字节的 CRC-32/ISO-HDLC，结果按小端存储 |

Python 格式：`struct.Struct("<4sBBHIQhhhhhhIIHHI")`，大小为 48。CRC 与 `zlib.crc32(frame[:44])` 一致：多项式 `0x04C11DB7`（反射实现 `0xEDB88320`）、初值和最终异或均为 `0xFFFFFFFF`、输入输出反射。ASCII `123456789` 的 CRC 为 `0xCBF43926`。

整帧参考向量：sequence=`0x12345678`、timestamp=`0x123456789ABCDEF0`，加速度 `-32768,32767,-1`，陀螺仪 `123,-456,789`，年龄 `624,499`，flags=`0x031F`，tick_late=`321`：

```text
49 4D 55 32 02 03 30 00 78 56 34 12 F0 DE BC 9A
78 56 34 12 00 80 FF 7F FF FF 7B 00 38 FE 15 03
70 02 00 00 F3 01 00 00 1F 03 41 01 69 BB 9C 26
```

## 质量标志和丢帧判断

| bit | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x0001` | 自上次成功入队以来出现输出节拍处理延迟超过 100 μs，或有整毫秒节拍被跳过 |
| 1 | `0x0002` | 自上次成功入队后出现过加速度 SPI 读取失败 |
| 2 | `0x0004` | 自上次成功入队后出现过陀螺仪 SPI 读取失败 |
| 3 | `0x0008` | 自上次成功入队后出现过 MCU 输出队列溢出 |
| 4 | `0x0010` | 该 USB 会话第一帧，应重建连续性基线 |
| 5 | `0x0020` | 所选加速度样本年龄超过 1000 μs |
| 6 | `0x0040` | 所选陀螺仪样本年龄超过 1000 μs |
| 7 | `0x0080` | 自上次成功入队后出现过加速度 DRDY 间隔/读取耗时异常 |
| 8 | `0x0100` | 自上次成功入队后出现过陀螺仪 DRDY 间隔/读取耗时异常 |
| 9 | `0x0200` | 自上次成功入队后出现过无法选出有效历史样本而跳过的输出 |
| 10–15 |  | 保留，当前为 0 |

活动 USB 会话内，`sequence` 在每个预定输出节拍递增。整毫秒节拍错过、历史样本缺失或入队失败也消耗序号；不会补发伪造的“赶进度”样本。没有两部分有效历史数据时不输出全零占位帧。读取暂时失败时仍可能选择之前的有效原始样本，SPI、年龄和异常标志可用于识别这种情况。

新 USB 会话序号重新从 0 计数，MCU 时间继续运行；MCU 重启后时间从零开始。录制开始时首个接收到的序号可为任意值，`0xFFFFFFFF → 0` 是正常回绕。序号缺口表示未收到对应输出帧，但不能单凭序号区分 MCU 队列满、错过节拍和主机接收/存储丢失，需结合 flags、CRC 和时间戳。

关闭/重开串口会重置软件队列并在新会话首帧置 bit 4。USB 已提交的在途包可能仍含少量前一会话数据，接收端按会话首帧标志重建基线。USB 背压过久会填满缓冲，固件丢弃新帧并在之后的成功入队帧报告丢失。

DRDY 间隔告警沿用原生速率阈值：加速度小于 468 μs 或大于 937 μs，陀螺仪小于 375 μs 或大于 750 μs；从该次中断时间戳起算，读取耗时超过 100 μs 也告警。这些阈值用于发现明显异常，不是抖动合格标准。

## 单位换算

- 加速度：`a_g = raw × 24 / 32768`；需要 m/s² 时乘 `9.80665`。
- 角速度：`omega_deg_s = raw × 2000 / 32768`；需要 rad/s 时乘 `π/180`。

这是名义灵敏度换算，不代替零偏、温漂和比例因子标定。

## 主机采集和解码

在线接收需要 pyserial，离线解码只用 Python 标准库：

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/imu_capture.py --port /dev/ttyACM0 --seconds 10 \
  --raw capture.bin --csv capture.csv --stats
python3 tools/imu_capture.py --input capture.bin --csv decoded.csv --stats
python3 tools/imu_capture.py --input capture.bin --csv units.csv --units --stats
```

Windows 把端口改为实际的 `COM5` 等。不指定 `--seconds` 时持续采集，Ctrl+C 正常关闭文件。程序不向设备发送命令。`--raw` 原样保存 USB 字节，包括坏帧和噪声；`--csv` 只保存校验通过的样本。

新协议每帧对应 CSV 一行，列为：

```text
time_us,sequence,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,
accel_age_us,gyro_age_us,accel_time_us,gyro_time_us,flags,tick_late_us,
protocol_version,legacy_sensor
```

新协议 `protocol_version=2`、`legacy_sensor` 为空。`--units` 保留上述列，再增加 `ax_g,ay_g,az_g,gx_deg_s,gy_deg_s,gz_deg_s` 六列。

`--stats` 输出 JSON：

- `imu.samples`、`sessions_observed`：有效六轴帧数和观测到的会话数。
- `imu.received_hz`：按共同时间戳计算的接收帧率，序号缺口会使其下降。
- `imu.sequence_hz`：将序号缺口计入后的网格节拍率，正常为 1000 Hz。
- `imu.period_us_min/max/mean/std`：仅对序号连续、时间递增、同一会话的共同时间戳间隔统计；标准差为总体标准差。正常间隔固定为 1000 μs。**这是输出时间网格统计，不代表原生 DRDY 或实际转换抖动为零。**
- `imu.accel_age_us`、`gyro_age_us`、`source_skew_us`、`tick_late_us`：分别统计样本年龄、两部分 DRDY 服务时间差的绝对值、输出中断响应延迟。各包含 `count/min/max/mean/std`。
- `imu.missing_sequences`、`duplicate_sequences`、`sequence_regressions`、`timestamp_regressions`、`flag_counts`：定位丢帧、重复和异常。
- `imu.source_repeats/source_regressions`：从年龄恢复的各部分源时间戳是否重复或倒退；会话切换时重建基线。`invalid_source_times` 表示帧内年龄大于共同时间、恢复出负源时间的异常数据。
- `parser.crc_errors/header_errors/bytes_discarded/trailing_bytes`：校验错误、错误候选头、重同步丢弃字节和文件尾不完整数据。随机噪声中出现 magic 也可能产生候选错误，不能把这些计数当作精确坏帧数。

评估输出节拍时查看共同时间戳，评估对齐情况时同时查看两路年龄、`source_skew_us` 和 `tick_late_us`。电脑的 USB `read()` 到达间隔受批量传输和操作系统调度影响，不适合判断板端采样抖动。

## 帧同步和旧版本兼容

接收端搜索 `IMU2`，验证 version=2、type=3、length=48，等够 48 字节后验证 CRC。失败时仅前进一个字节重新搜索，成功时消费 48 字节。不要依赖 USB 一次读取恰好一帧。随附解析器支持任意分段、噪声、错误帧后的重同步，内部缓冲上限为 4096 字节。

随附主机程序也保留了旧 `IMU1` 32 字节版本的解码能力，便于读取此前录制的文件；新固件只发送 `IMU2`。旧帧分别统计到 JSON 的 `legacy.accel`、`legacy.gyro`，不会混入 `imu` 的 1000 Hz 合并帧统计。旧帧导出 CSV 时 `protocol_version=1`、`legacy_sensor=accel/gyro`，只填对应三轴和源时间列，另一个传感器及新协议专有列留空，不伪造六轴配对。旧版 bit 3 是会话标志，而新版 bit 3 是队列溢出、bit 4 才是会话标志。
