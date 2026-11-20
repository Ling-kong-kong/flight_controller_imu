# 录制与导出文件结构说明

本文对应当前 `viewer` 的保存和导出实现。用于回答：文件存在哪里、每个文件保存什么、如何找到六轴／图像／姿态，以及归档时应保留哪些内容。

> **录制与导出是独立阶段，现已自动串联。** 操作流程为“开始录制 → 完成录制 → 原始保存完成 → 自动导出”，无需再手动导出。录制保存原始数据；导出读取已完成的录制，生成图像关联表与 EuRoC-shaped 数据集，不修改原始会话。统一时间为电脑 `perf_counter_ns()` 读取完成时间，**不是图像曝光时间，也不是 IMU 物理采样时间；曝光同步误差未验证**。

## 1. 保存位置与操作流程

1. **连接预览**：显示图像、波形及姿态，不创建录制会话；内存波形历史不等于录制文件。
2. **开始录制**：只选择一个保存根目录；在其中新建一个 `session_年月日_时分秒_微秒` 总会话文件夹，原始数据写入其中的 `raw/`。
3. **完成录制**：等待已有队列排空、文件关闭及最终化完成。GUI 立即弹出进度小窗口，先显示保存收尾，再显示导出；保持连接并继续预览。
4. **自动导出**：原始保存结束后自动在后台运行，在本次总会话文件夹内创建与 `raw/` 并列的 `export/` 子目录；进度小窗口显示正在导出，成功后自动关闭，主窗口保留成功路径；失败时小窗口保留错误并可关闭后重试，无需手动点击导出。导出完成前不能开始下一次录制。

默认保存根目录为 `D:/work_code/Bixocean/flight_controller_imu/viewer/recordings/`，可在“设备 / 保存设置”中选择其他位置。**原始数据和导出结果位于同一个总会话文件夹内，分别存放在 `raw/` 和 `export/`。** 录制开始时固定本次位置，录制、收尾、导出期间禁用目录修改。默认 `recordings/` 已被本地 `.gitignore` 忽略，自选到其他位置的目录不一定被 Git 忽略。

每次录制创建独立会话，不把本次连接中录制前的预览历史补写进去。本次总会话文件夹内的 `export/` 已存在（包括未完成目录）时，自动改用 `export_001/`、`export_002/` 等，不覆盖。录制期间断开连接、设备故障或关闭主窗口也会在安全收尾后尝试自动导出；关闭窗口会等待导出结束。错误会话仍需检查质量报告，不能因导出成功就认为数据无损。

导出失败时原始会话仍保留，界面显示错误和相关路径。修复问题后，可点击 **重新导出…**，选择已完成的总会话文件夹或其中的 `raw/` 重试，导出结果仍放在同一总文件夹内；也支持旧会话（结果放在旧原始目录旁的 `原会话名_euroc/`，必要时追加编号，不搬移原始数据）；不需要重新录制。

下文的日期、时间戳、帧号均为命名示例，不代表已产生实机数据。目录名中的系统日期只用于命名，不参与时间对齐。

## 2. 原始录制会话目录

以默认保存位置、正常完成录制为例：

```text
viewer/
└── recordings/
    └── session_20261120_140000_123456/
        ├── export/              # 原始保存完成后自动生成，详见第 3 节
        └── raw/
            ├── session.json
            ├── devices.json
            ├── usb.bin
            ├── usb_batches.csv
            ├── imu.csv
            ├── frames.csv
            ├── cam0/
            │   ├── 000000000.png
            │   ├── 000000001.png
            │   └── ...
            ├── cam1/
            │   ├── 000000000.png
            │   ├── 000000001.png
            │   └── ...
            ├── events.jsonl
            ├── aligned_frames.csv
            ├── clock_models.json
            ├── clock_residuals.csv
            ├── quality.json
            ├── failure.json          # 条件文件：最终化等异常时尝试写入
            └── verification.json     # 条件文件：单独运行 verify 且成功后生成
```

`aligned_frames.csv`、`clock_models.json`、`clock_residuals.csv` 在最终化时生成；`quality.json` 在停止收尾时保存。录制进行中或异常中断时，文件可能尚未生成或不完整，不能只凭目录存在就认为录制完整。

### 2.1 原始数据与索引

| 文件／目录 | 内容与用途 |
| --- | --- |
| `usb.bin` | 进入本次录制的原始 USB 接收字节，包括传感器帧、收到的控制 ACK，以及无法解码／CRC 错误／首尾不足完整帧的字节。可用于重新解码和审计。 |
| `usb_batches.csv` | 每次非空串口读取一行，记录原始字节偏移、长度、电脑读取起止时间及解码帧数；没有解出完整帧的批次也保留。 |
| `imu.csv` | 每条 CRC 有效的六轴合并数据帧一行，包含原始六轴、H743 时间、接收时间、序号、质量标志、物理量及姿态；不按相机帧率降采样，不按姿态有效性删除六轴行。 |
| `frames.csv` | 每个已提交双目帧一行，记录媒体时间、电脑读取区间、统一时间、原拼接帧尺寸和左右目相对路径。 |
| `cam0/`、`cam1/` | 输入拼接图的左、右半幅，保存为彩色无损 PNG。文件名为九位补零 `frame_id`，例如同一双目帧的 `cam0/000000000.png` 与 `cam1/000000000.png`。默认每半幅 1280×720，实际以配置及已保存图像为准。 |

**原始录制的 `imu.csv` 不包含图片关联栏**，IMU 与图片分别保存，通过各自索引保留时间信息。你要求的“六轴 → 图片信息 → 四元数／欧拉角”合并布局在导出的 `imu_image.csv` 中提供。

### 2.2 关键 CSV 字段

#### `imu.csv`

以下按含义分组，不表示源 CSV 的实际列顺序：

| 字段 | 含义 |
| --- | --- |
| `sample_id`、`batch_id` | 本次录制的 IMU 行编号、完成解码的 USB 读取批次编号。 |
| `unified_timestamp_ns` | 对应批次的 `host_read_end_ns`，即统一电脑接收时间，整数纳秒。 |
| `sequence`、`timestamp_us`、`timestamp_ns` | 板端序号与共同 H743 时间；纳秒由微秒用整数乘 1000 得到，不覆盖原始时间。 |
| `ax_raw`、`ay_raw`、`az_raw`、`gx_raw`、`gy_raw`、`gz_raw` | 原始三轴加速度、三轴角速度整数。 |
| `accel_age_us`、`gyro_age_us`、`accel_time_us`、`gyro_time_us` | 样本年龄与由共同时间减去年龄得到的源样本时间。 |
| `flags`、`tick_late_us`、`protocol_version` | 设备质量标志、输出节拍延迟与协议版本。 |
| `ax_g/ay_g/az_g`、`ax_m_s2/ay_m_s2/az_m_s2` | 加速度名义换算，单位分别为 g 和 m/s²。 |
| `gx_deg_s/gy_deg_s/gz_deg_s`、`gx_rad_s/gy_rad_s/gz_rad_s` | 角速度名义换算，单位分别为 °/s 和 rad/s。 |
| `qw`、`qx`、`qy`、`qz` | 同条 H743 数据帧提供的四元数，标量在前，机体 FRD → 世界 NED。 |
| `roll_deg`、`pitch_deg`、`yaw_deg`、`attitude_valid` | 板端 Euler 3-2-1 角度（度）及姿态有效标志。姿态无效时七个姿态数值字段为空、有效标志为 0；不把无效占位值保存成有效姿态。 |

当前名义换算为原始加速度 × `24/32768` 得 g，原始角速度 × `2000/32768` 得 °/s；g 转 m/s² 使用 `9.80665`，°/s 转 rad/s 使用 `π/180`。不额外对原始六轴扣零偏、旋转坐标或根据图片反算姿态。

姿态来自板端，不是从图片解算得到；相对 yaw 会漂移，不代表真北。安装轴向尚未实板核验。重校准可改变 yaw 参考，应结合 `events.jsonl` 分段解读。旧会话缺少姿态列时不补算或伪造。

#### `usb_batches.csv`

```text
batch_id, raw_offset, byte_count, host_read_start_ns,
host_read_end_ns, frame_count, last_mcu_ns
```

`raw_offset` 和 `byte_count` 指向 `usb.bin` 的字节范围。**同一 USB 批次内多条 IMU 共享同一接收时间**，不人为插值生成不同到达时刻。跨读取的拆包帧归入完成解码的批次。无完整 IMU 帧时 `last_mcu_ns` 为空。

#### `frames.csv`

```text
frame_id, camera_media_ms, camera_media_ns,
host_read_start_ns, host_read_end_ns, unified_timestamp_ns,
width, height, cam0, cam1
```

`width/height` 为原始双目拼接帧的尺寸；左右各取一半宽度。`camera_media_ns` 无有效值时可为空，媒体时钟域与曝光语义未验证，不参与当前统一时间计算。`cam0/cam1` 是相对于原始数据目录 `raw/` 的路径，不是相对于外层总会话文件夹。

### 2.3 元数据、时间方案与状态

| 文件 | 用途 |
| --- | --- |
| `session.json` | schema、请求配置、录制生命周期、时间基准、姿态坐标及限制声明。完成后 `finalized=true`，状态为 `stopped` 或 `stopped_error`；错误停止不等于完整无损。 |
| `devices.json` | USB 身份、相机 PnP 清单、OpenCV 版本及环境信息，不是相机内外参标定。相机实际输出等诊断见 `quality.json`。 |
| `events.jsonl` | 每行一个 JSON 事件，含电脑单调时间、事件类型及消息；用于故障、操作及重校准／yaw 参考边界追溯，可能为空。 |
| `aligned_frames.csv` | 当前方案的表头为 `frame_id,camera_media_ns,unified_timestamp_ns,status`；统一时间直接取图像读取结束时间，状态为 `host_receive_time`，不修改 `frames.csv`。 |
| `clock_models.json` | 当前时间方案声明与诊断。`timestamps_valid=true`、`models_valid=false`、`models={}`、`clock_fitting_used=false` 表示直接使用接收时间，不是拟合失败。 |
| `clock_residuals.csv` | 当前接收时间方案仅保留表头，没有拟合残差数据；旧会话可能包含旧模型残差。残差不等于曝光同步精度。 |
| `quality.json` | 已写计数、实测速率、CRC／连续性、队列／磁盘／内存遥测、未保存计数及故障。`done` 是最终化前快照，可能为 false，不能据此判断后台还在运行。 |
| `failure.json` | 最终化等失败时尝试保存错误及堆栈；磁盘已不可写时也可能无法生成，不是所有错误都会有此文件。 |
| `verification.json` | 单独运行 `verify` 成功后的审计报告，包括 USB 重解码、CSV／双目索引核验、文件 SHA256 等。导出内部执行只读校验，不会自动生成此文件。 |

## 3. 导出目录

以默认保存根目录 `viewer/recordings/` 下同一总会话文件夹中的自动导出结果为例：

```text
viewer/
└── recordings/
    └── session_20261120_140000_123456/
        ├── raw/                 # 完整原始数据，详见第 2 节
        └── export/
            ├── imu_image.csv
            ├── export.json
            ├── README.txt
            ├── mav0/
            │   ├── cam0/
            │   │   ├── data.csv
            │   │   └── data/
            │   │       ├── 123456789000_000000000.png
            │   │       └── ...
            │   ├── cam1/
            │   │   ├── data.csv
            │   │   └── data/
            │   │       ├── 123456789000_000000000.png
            │   │       └── ...
            │   └── imu0/
            │       └── data.csv
            ├── provenance/
            │   ├── session.json
            │   ├── devices.json
            │   ├── imu.csv
            │   ├── usb_batches.csv
            │   ├── frames.csv
            │   ├── aligned_frames.csv
            │   ├── clock_models.json
            │   ├── clock_residuals.csv
            │   ├── quality.json
            │   └── events.jsonl
            └── EXPORT_INCOMPLETE      # 仅导出进行中／中断时存在；成功后删除
```

导出索引中的所有相对路径均以 `export/`（或 `export_001/` 等重试目录）为基准。`provenance/` 是原始文件的原样副本，其中的原始图像路径仍相对于源 `raw/`。

`provenance/` 中列出的文件仅在源会话存在时复制，保持源内容，不重排其中的 `imu.csv`。当前实现不复制源会话的 `verification.json` 或 `failure.json`。

### 3.1 `imu_image.csv`：按你的要求排列的全量合并表

| 分组顺序 | 列 |
| --- | --- |
| ① 原始六轴 | `ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw` |
| ② 图片信息 | `image_frame_id, image_timestamp_ns, image_minus_imu_ns, cam0, cam1` |
| ③ 四元数／欧拉角 | `qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, attitude_valid` |
| ④ 补充 IMU 信息 | 其余全部源 IMU 字段，按源表中的相对顺序保留，包括编号、时间戳、序号、质量及物理量。旧会话缺少统一时间列时依据批次接收时间补充。 |

- **一条 IMU 一行，全部保留**，不只输出匹配成功的数据，也不按图像频率降采样。
- **未匹配图像：五个图片字段均为空**，不是 0；六轴与有效姿态仍保留。
- **姿态无效：姿态数值为空**，`attitude_valid=0`，与是否匹配图像无关。旧会话源表缺少姿态列时不凭空增加姿态列或值。
- `image_timestamp_ns` 为图像电脑读取结束时间；`image_minus_imu_ns` 为图像接收时间减去 IMU 接收时间的有符号差值。
- `cam0/cam1` 为相对于导出目录的灰度图片路径，例如 `mav0/cam0/data/123456789000_000000000.png`。
- 当前关联按相机顺序，在容差内找最近的尚未匹配 IMU，一对一；等距选择较早 IMU，同批同时间选择较早 `sample_id`。GUI 默认容差 **20 ms**，CLI 可配置；**容差不是同步精度**。
- 未匹配到 IMU 的图像仍保存在相机目录与索引中；不会凭空创建无 IMU 的合并表行。

实际列顺序和分组记录在 `export.json` 的 `imu_image_columns`、`imu_image_column_groups` 中。

### 3.2 EuRoC-shaped 数据与导出报告

| 文件／目录 | 内容 |
| --- | --- |
| `mav0/cam0/data/*.png`、`mav0/cam1/data/*.png` | 源会话中全部已索引双目帧转换的灰度 PNG，不仅保留匹配成功的图片。 |
| 两目的 `data.csv` | 两列 `#timestamp [ns],filename`；时间为图像接收时间，文件名为 `接收时间戳_九位frame_id.png`，避免相同时间戳覆盖图片。两目索引对应。 |
| `mav0/imu0/data.csv` | 七列：`时间戳, gx, gy, gz, ax, ay, az`；角速度 rad/s、加速度 m/s²。所有 CRC 有效合并 IMU 帧保留，不按 flags 删除，不追加姿态列。 |
| `export.json` | 时间基准、列分组、匹配策略和容差、IMU／图像数量、未匹配数量、重复时间戳计数、源路径及参与审计文件的 SHA256 等。 |
| `README.txt` | 随导出附带的格式、时间及同步限制说明。 |
| `provenance/` | 原始索引、IMU CSV、配置、质量及时间方案文件副本，方便追溯；不是完整原始备份。 |

**仅提供 EuRoC-shaped 结构，不保证所有 EuRoC 消费者都能直接读取**：USB 批次接收时间可重复，程序不伪造严格递增时间；图像文件名含帧号。不生成 `sensor.yaml`，也不伪造内参、外参或同步精度。`strict_euroc_timestamp_compatibility` 只是时间重复检查结果，不是整个数据集已标定或完整兼容的证明。

## 4. 数据之间如何对应

```text
原始 USB 字节 usb.bin
    └─ usb_batches.csv：batch_id + 字节范围 + 电脑读取区间
        └─ imu.csv：sample_id + batch_id + 六轴 + 板端姿态 + 时间

双目彩色 PNG cam0/、cam1/
    └─ frames.csv：frame_id + 两目路径 + 媒体时间 + 电脑读取区间

停止录制并最终化
    └─ aligned_frames.csv + clock_models.json + quality.json

自动导出（原始保存完成后触发，不重新采集，不修改源文件）
    ├─ imu_image.csv：每条 IMU + 可为空的图片关联 + 板端姿态
    ├─ mav0/：全部 IMU 七列表 + 全部已索引双目灰度图及索引
    └─ export.json + README.txt + provenance/
```

IMU 与相机是并行采集，不是“先采六轴、再拍图片、最后由图片算姿态”的处理流水线。数据频率不同，合并表出现大量空图片栏是正常的一对一关联结果。

## 5. 校验、异常与归档建议

- **完整归档应保留整个总会话文件夹，将 `raw/` 与 `export/`（及需要保留的重试目录）一起保存。** 导出不包含 `usb.bin` 和原始彩色图片；只保留导出目录不能重建完整原始录制。
- `EXPORT_INCOMPLETE` 存在时，禁止将该导出当作完成结果。自动导出和 GUI 重新导出会为同名目标追加数字后缀；显式 CLI `export` 拒绝覆盖已有目录，须指定新的完整目标路径。
- `stopped_error`、`failure.json`、队列溢出／未保存计数等需要结合检查。双目都写入后才提交图像索引，故障可能遗留未索引 PNG；不要按目录文件数量自行补造索引。
- `json_write` 写入时可能短暂出现 `.json.tmp` 文件；异常中断后可能遗留，不能代替正式 JSON 文件。
- CSV／原始字节关闭时 flush + fsync，元数据采用临时文件替换，但不保证断电事务原子性。不要强杀仍在保存／导出的程序；原始保存完成和导出完成是两个先后发生的状态。
- 所有统一纳秒时间是电脑单调时钟，不是 UTC；不得将不同电脑、不同系统启动周期的数据直接当作同一绝对纪元合并。同批时间重复不表示 MCU 重复采样。

可从 `viewer` 目录运行以下离线命令；将示例会话名替换成实际名称：

```powershell
# 校验；可以选择总会话文件夹，成功后在其中 raw/ 内生成 verification.json
.\.venv\Scripts\python.exe -m imu_viewer verify .\recordings\session_20261120_140000_123456

# 如需检查每一张左右目图片是否可解码，增加 --decode-all
.\.venv\Scripts\python.exe -m imu_viewer verify .\recordings\session_20261120_140000_123456 --decode-all

# 可选：显式重导出；目标必须尚不存在，CLI 不会自动追加数字后缀
.\.venv\Scripts\python.exe -m imu_viewer export .\recordings\session_20261120_140000_123456\raw .\recordings\session_20261120_140000_123456\export_001 --match-tolerance-ms 20
```

旧 schema 1 会话可能包含以 H743 为基准的拟合模型和 `estimated_mcu_ns` 等旧列；源文件与 provenance 保持原样，当前导出仍依据已保存的电脑读取结束时间。详细时间语义及界面操作见 [README](README.md)。
