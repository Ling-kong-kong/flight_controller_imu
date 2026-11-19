# IMU2 USB 六轴与姿态协议（版本 3，固定 1000 Hz）

USB 为 Full Speed CDC ACM 虚拟串口。打开串口并置 DTR 后持续接收二进制数据，无需发送命令。115200 等波特率只是 CDC 兼容参数，不限制 USB 实际带宽。固件仅发送 IMU2 v3/type 3 组合帧，不混入调试文本、温度、气压或磁力计数据。

**每秒 1000 帧，每帧 76 字节，名义应用数据量为 76000 B/s（76 kB/s，不含 USB 开销）**，同时包含原始六轴、共同时间戳、质量标志、四元数和 roll/pitch/yaw。USB 包边界及一次串口 read() 的长度均不是协议帧边界。

## 时间对齐与原始轴

MCU 使用同一个 1 MHz TIM2 时基计时和安排输出，共同时间戳为绝对网格 t0+n×1000 μs。BMI088 内部仍为加速度 **1600 Hz、±24 g**、陀螺仪 **2000 Hz、±2000 °/s**；每个输出时刻分别选择**不晚于该时刻的最新原始样本**，不做插值或补造赶进度样本。TIM2 只选样、快照并放入有界融合队列，主循环按序融合、编码、计算 CRC 并处理 USB。

BMI088 两部分内部时钟独立，未确认具备官方同步功能所需的陀螺 DRDY → 加速度中断输入连线。这里统一的是**输出节拍与时间基准，不是物理转换时刻**：

~~~text
accel_time_us = timestamp_us - accel_age_us
gyro_time_us  = timestamp_us - gyro_age_us
source_skew_us = abs(accel_age_us - gyro_age_us)
~~~

源时间是 DRDY 被 MCU 服务的时刻，包含中断响应延迟，不是精确 ADC 转换时间或 BMI088 sensortime。健康时年龄通常分别在约一个 625 μs、500 μs 原生周期以内；还受 SPI 读取和中断响应影响，两部分滤波群延迟也可能不同。加速度使用 Normal 滤波（XY 280 Hz、Z 245 Hz），陀螺使用 2000 Hz / 230 Hz 配置。

**原始六轴保留芯片寄存器轴和 int16 整数，不扣零偏、不旋转。** 名义换算：

- 加速度：a_g = raw × 24 / 32768；乘 9.80665 得 m/s²。
- 角速度：omega_deg_s = raw × 2000 / 32768；乘 π/180 得 rad/s。

这些换算不替代零偏、温漂、比例因子标定；融合内部的零偏扣除不会改变线上原始值。

## v3 每帧 76 字节

所有多字节字段为**小端序**，原始轴为二进制补码 int16，姿态为 IEEE-754 float32；没有行尾、转义或 ASCII 日志。前 44 字节字段位置沿用 v2，但 version/length 更新。

| 偏移 | 字节数 | 字段 | 定义 |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII IMU2，49 4D 55 32 |
| 4 | 1 | version | uint8，3 |
| 5 | 1 | type | uint8，3 |
| 6 | 2 | length | uint16，76 |
| 8 | 4 | sequence | uint32，每个输出节拍递增，模 2³² 回绕 |
| 12 | 8 | timestamp_us | uint64，该节拍的共同参考时间 |
| 20 / 22 / 24 | 各 2 | ax_raw / ay_raw / az_raw | 加速度芯片寄存器轴，int16 |
| 26 / 28 / 30 | 各 2 | gx_raw / gy_raw / gz_raw | 陀螺芯片寄存器轴，int16 |
| 32 / 36 | 各 4 | accel_age_us / gyro_age_us | uint32，源 DRDY 服务时间距共同时间的差 |
| 40 | 2 | flags | uint16，见下表 |
| 42 | 2 | tick_late_us | uint16，定时中断服务距节拍的延迟，超过 65535 饱和 |
| 44 / 48 / 52 / 56 | 各 4 | qw / qx / qy / qz | float32，机体到世界的单位四元数，标量在前 |
| 60 / 64 / 68 | 各 4 | roll_deg / pitch_deg / yaw_deg | float32，Euler 3-2-1，单位度 |
| 72 | 4 | crc32 | 前 72 字节的 CRC-32/ISO-HDLC，小端存储 |

Python 格式：struct.Struct("<4sBBHIQhhhhhhIIHH7fI")。CRC 等于 zlib.crc32(frame[:72])：多项式 0x04C11DB7（反射实现 0xEDB88320），初值/最终异或均为 0xFFFFFFFF，输入输出反射；ASCII 123456789 → 0xCBF43926。

v3 编码/CRC 参考向量（下列十六进制行直接拼接）：

~~~text
494d553203034c0078563412f0debc9a785634120080ff7fffff7b0038fe1503
70020000f3010000000841010000803f000000000000000000000000
00000000000000000000b442187ba85c
~~~

其 sequence=0x12345678，timestamp=0x123456789ABCDEF0，加速度=-32768,32767,-1，陀螺=123,-456,789，年龄=624,499，flags=0x0800，tick_late=321，q=1,0,0,0，Euler=0,0,90，CRC=0x5CA87B18。**该向量刻意使用不一致的四元数/Euler，仅验证字节布局和 CRC，不是物理姿态参考。**

## 姿态坐标、校准与有效性

- 机体 **FRD**：X 前、Y 右、Z 下；世界 **NED**：X 北、Y 东、Z 下，但六轴不能观测真北，本期水平航向是校准时定义的相对方向。
- qw,qx,qy,qz 表示**机体 → 世界**旋转；Euler 采用 R = Rz(yaw) Ry(pitch) Rx(roll) 的 3-2-1 顺序，右手正方向，单位度。roll/yaw 通常位于 ±180°，pitch 位于 ±90°；pitch 接近 ±90° 时 Euler 存在奇异性，应优先使用四元数。
- 仅融合输入暂按历史 ROTATION_PITCH_180 做 X/Z 取反，配置在 fusion.h 的 FUSION_INSTALLATION_PITCH_180。**这是安装假设，不是已验证的实板方向；原理图及历史配置不能证明板轴。** 必须六面静置、绕已知机体轴转动核验；若不符，修正融合安装旋转并重测，不能改原始六轴轴定义来迁就姿态。
- 不接入 BMM150。每次开机或异常后重新静置校准完成时，估计陀螺零偏、用重力初始化倾角、yaw 置零。yaw 是**允许漂移的相对航向**，不是磁航向/真北。正常 USB 断开重连不重置姿态或重新归零。
- 固定静置参数：200 ms 窗口内原始加速度各轴标准差 ≤0.02 g、**30 Hz 检测专用低通后的角速度各轴标准差 ≤0.5 °/s**，当前原始角速度模长 <5 °/s、原始加速度模长距 1 g ≤0.1 g。低通在固定 1 kHz 网格运行，单极递推 y += 0.1717958187 × (x-y)，首个合格样本直接初始化；只改变静置陀螺标准差，不改变原始 USB 六轴、零偏的合格原始值均值、姿态积分或加速度重力门控。窗口填满后连续约 2 秒静置才完成校准，典型约 **2.2 秒**，不是新固件实测上电结论。普通运动只清连续静置累计，保留滤波/统计窗；坏/过期样本、重复/回退陀螺或不连续节拍清检测器并重新建立 200 ms 窗口，整体融合重置也清检测器。已有效时重建加速度统计窗期间暂仅积分合格陀螺。实板仍须核验运动误接受，尤其不能区分阈值内恒定慢 yaw 与零偏。实现见 `firmware/src/fusion.c`，回放工具为 `tools/imu_fusion_replay.py`。线协议版本、布局及 CRC 不变。
- Mahony Kp=2 s⁻¹、Ki=0。加速度模长/短时波动不适合重力修正时，只积分陀螺；陈旧、错误、重复/回退的陀螺源样本不积分，当前帧姿态无效。短暂跳节拍不追补缺失区间，仅积分当前至多 1 ms。融合队列丢帧或数据中断 **>10 ms** 时失效、重置并重新校准。

**只有 bit 11 置位，且校准/重置位未置位，才可作为有效姿态。** 本项目接收端还检查源时间、陀螺年龄 ≤1000 μs/无 SPI 错误、七个有限浮点数以及四元数归一化（模平方距 1 ≤0.01）。无效时线上姿态数值只是占位，不保证有物理意义；不得显示为有效测量，CSV 姿态数值必须留空。旧 v2 同样没有有效姿态。姿态无效并不自动否定原始六轴，原六轴仍按自身质量标志判断。

## 标志、会话与丢帧

| bit | 掩码 | 含义 |
|---:|---:|---|
| 0 | 0x0001 | 输出处理迟到 >100 μs 或跳过整毫秒节拍 |
| 1 | 0x0002 | 加速度 SPI 读取失败 |
| 2 | 0x0004 | 陀螺 SPI 读取失败 |
| 3 | 0x0008 | 融合队列或 USB 发送队列溢出 |
| 4 | 0x0010 | USB 会话首帧，重建连续性基线 |
| 5 | 0x0020 | 所选加速度年龄 >1000 μs |
| 6 | 0x0040 | 所选陀螺年龄 >1000 μs |
| 7 | 0x0080 | 加速度 DRDY 间隔/读取耗时异常 |
| 8 | 0x0100 | 陀螺 DRDY 间隔/读取耗时异常 |
| 9 | 0x0200 | 无有效历史样本，曾跳过输出 |
| 10 | 0x0400 | 校准中，提示静置 |
| 11 | 0x0800 | 当前帧姿态有效 |
| 12 | 0x1000 | 融合重置事件，需重新校准 |
| 13–15 | | 保留，当前为 0 |

bit 5/6 与姿态状态是当前帧状态；其他采集异常为待报告事件，会随快照向后传递。发送失败会重新挂起原有采集异常位，不能将这些位理解为错误恰好发生在当前节拍。重置事件可能在 USB 关闭或丢帧期间发生，接收端不能只依赖 bit 12，应总是检查 bit 11/10。

每个预定输出节拍消耗 sequence，错过节拍、无历史样本、融合丢帧和发送丢帧均保留缺口。没有两路有效历史数据时不输出全零伪造样本；SPI 暂时失败仍可能输出之前的原始样本并报告错误/年龄。0xFFFFFFFF → 0 正常回绕，录制起始序号可为任意值。

关闭/重开 CDC、USB 卸载或挂起只清空发送队列并重建发送会话；新会话重新计数，MCU 时间继续运行，融合持续更新。MCU 重启才使时间重新开始。USB 在途包可能残留少量旧会话数据，按 bit 4 重建基线。USB 背压只丢弃新的发送帧，不重置融合；若主循环本身阻塞造成融合队列丢帧，则需重校准。

缓冲：每传感器 8 个原始历史样本、64 个融合快照、512×76 字节发送队列、TinyUSB 4096 字节发送缓冲。通过 SWD 查看 g_imu_diagnostics：fusion_queue_drops（含溢出后废弃的积压）、fusion_high_water、fusion_resets、ring_drops、timer_skipped、SPI/DRDY/年龄/延迟等计数。bit 3 或序号缺口本身不能区分各类丢失。查看 g_last_imu_frame 时检查 CRC 并重试；调试暂停本身会造成中断/重置。

DRDY 告警阈值：加速度间隔 <468 或 >937 μs，陀螺 <375 或 >750 μs；读取耗时 >100 μs 也告警。这不是抖动合格标准。固定网格周期标准差为零，不表示原生采样抖动为零。

## 主机采集与 CSV

~~~bash
python3 -m pip install -r tools/requirements.txt
python3 tools/imu_capture.py --port /dev/ttyACM0 --seconds 10 --raw capture.bin --csv capture.csv --stats
python3 tools/imu_capture.py --input capture.bin --csv decoded.csv --stats
python3 tools/imu_capture.py --input capture.bin --csv units.csv --units --stats
~~~

Windows 使用实际 COM 端口。不指定 --seconds 时持续采集，Ctrl+C 正常关闭。--raw 保存全部 USB 原始字节（包括噪声/坏帧），--csv 仅输出 CRC 通过的帧，离线解析只需 Python 标准库。

CLI CSV 列：

~~~text
time_us,sequence,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,
accel_age_us,gyro_age_us,accel_time_us,gyro_time_us,flags,tick_late_us,
protocol_version,legacy_sensor,attitude_valid,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg
~~~

v3 的 protocol_version=3、legacy_sensor 为空；attitude_valid 为 0/1，无效或 v2 的七个姿态数值留空。--units 另加 ax_g/ay_g/az_g/gx_deg_s/gy_deg_s/gz_deg_s。viewer 的 imu.csv 还增加 batch/sample/接收时间与 SI 等列；session.json schema 3 记录协议、坐标、相对航向和未验证安装旋转。重解 usb.bin 校验覆盖新增姿态列；旧 schema 2 无姿态列仍可校验。EuRoC 的 mav0/imu0/data.csv 保持标准六轴七列，imu_image.csv 保留姿态列。

--stats JSON 保留：

- imu.samples / sessions_observed：CRC 有效组合帧数/会话数。
- imu.received_hz / sequence_hz：按共同时间计算的收到帧率/含缺号的网格节拍率。
- imu.period_us_min/max/mean/std：仅同会话连续序号、时间递增的网格周期，不表示 DRDY 抖动。
- accel_age_us / gyro_age_us / source_skew_us / tick_late_us：count/min/max/mean/std。
- missing_sequences / duplicate_sequences / sequence_regressions / timestamp_regressions / flag_counts：连续性和质量诊断。
- source_repeats / source_regressions / invalid_source_times：源时间重复、回退及年龄大于共同时间等错误。
- parser.crc_errors / header_errors / bytes_discarded / trailing_bytes：候选 CRC/头错误、重同步丢弃和尾部不完整字节；不是精确坏帧计数。

USB read() 到达间隔受批量传输/系统调度影响，不可用于判定板端采样抖动。

## 重同步与旧版兼容

搜索 IMU2，按 version/type/length 选择合法组合：**v3/type3/76** 或 **v2/type3/48**，等足长度后检验相应 CRC；失败仅前进一个字节继续搜索，成功消费完整一帧。支持任意拆包、粘包、混合版本、噪声及坏帧后重同步，内部缓冲小于 4096 字节。接收端不得只因看到 magic 就接受数据。

v2 的前 44 字节布局相同（version=2、length=48），偏移 44 为 CRC、覆盖前 44 字节，Python 格式 <4sBBHIQhhhhhhIIHHI；无四元数/Euler 或有效姿态。保留其离线解析，不对旧录制生成姿态。v2 原始参考向量：

~~~text
49 4D 55 32 02 03 30 00 78 56 34 12 F0 DE BC 9A
78 56 34 12 00 80 FF 7F FF FF 7B 00 38 FE 15 03
70 02 00 00 F3 01 00 00 1F 03 41 01 69 BB 9C 26
~~~

CLI 另兼容 IMU1 32 字节旧文件（viewer 不接收 IMU1）。旧帧在 JSON 中分别统计到 legacy.accel/gyro，不混入组合帧统计；CSV protocol_version=1、legacy_sensor=accel/gyro，只填对应三轴/源时间，另一传感器和不适用字段留空，不伪造配对。注意 IMU1 bit 3 为会话位，而 IMU2 bit 3 为溢出、bit 4 为会话位。

## 验证边界

软件回归与回放不能替代实板验收。发布前仍须检查六面静置与三轴已知方向转动、约 2 秒静置校准、运动不误触发校准、1 kHz / 76 kB/s 连续传输、丢帧诊断及 USB 重连。历史 validation.json、release/BUILD_INFO.json 及旧上位机实测记录不代表本功能通过；本轮采集数据、烧录日志及验证报告仅在本地保留。
