# 保存文件结构说明

本文说明当前上位机的保存位置、文件内容、六轴／图像／姿态关联与归档方法。

> **操作流程：开始录制 → 完成录制 → 保存完成。** 不再运行自动或手动导出。保存完成即包含全量六轴、可为空的图片关联、板端四元数／欧拉角及完整时间字段。统一时间仍为电脑 `perf_counter_ns()` 读取完成时间，**不是图像曝光时间或 IMU 物理采样时间；曝光同步误差未验证**。

## 1. 保存位置与操作流程

1. **连接预览**：显示图像、波形与姿态，不创建录制会话；内存波形历史不等于录制文件。
2. **开始录制**：选择保存根目录，创建独立的 `session_年月日_时分秒_微秒/raw/`，只保存本次录制期间收到的数据，不补写此前的预览历史。
3. **完成录制**：弹出“正在保存”窗口；后台排空队列、完成已有 PNG 写盘任务、关闭文件并生成时间索引与全量 IMU 关联表。设备保持连接、预览继续；保存成功后弹窗自动关闭，主窗口显示路径。
4. **保存失败**：错误窗口明确显示“保存失败”、原因与已写入路径，提示可能不完整；允许关闭提示，不自动删除数据。没有导出／重试导出／取消导出阶段。

默认根目录为 `D:/work_code/Bixocean/flight_controller_imu/viewer/recordings/`，可在“设备 / 保存设置”选择其他位置。目录在本次录制及收尾期间固定；保存完成前不能开始下一次录制。默认 `recordings/` 已被本地 `.gitignore` 忽略，自选路径不一定被 Git 忽略。

所有结果在同一个总会话文件夹内，不新建 `export/` 或 EuRoC 数据集，不对原始图片进行第二次解码、转换、复制或编码。但仍需等待已提交写盘、CSV 关联表与文件同步，不能保证点击完成后瞬间结束。

录制期间断开连接、设备故障或关闭主窗口时，仍执行安全停止与保存收尾；有采集故障的会话不能因为收尾完成就认为完整无损。旧版本已有的导出目录保持不动，新录制不再生成。

下列目录名、时间与帧号仅作示例，不代表新实机采集。系统日期只用于命名，不参与对齐。

## 2. 保存会话目录

以正常完成录制为例：

```text
viewer/
└── recordings/
    └── session_20261120_140000_123456/
        └── raw/
            ├── session.json
            ├── devices.json
            ├── usb.bin
            ├── usb_batches.csv
            ├── imu.csv
            ├── imu_image.csv         # 全量 IMU 主表，图片／姿态可为空
            ├── association.json      # 关联规则、数量、列分组与限制
            ├── frames.csv
            ├── cam0/
            │   ├── 000000000.png
            │   └── ...
            ├── cam1/
            │   ├── 000000000.png
            │   └── ...
            ├── events.jsonl
            ├── aligned_frames.csv
            ├── clock_models.json
            ├── clock_residuals.csv
            ├── quality.json
            ├── failure.json          # 条件文件：最终化等异常时尝试写入
            └── verification.json     # 条件文件：单独 verify 成功后生成
```

`imu_image.csv`、`association.json` 和时间索引在保存收尾时生成。录制中或异常中断后文件可能缺失或不完整，不能只凭目录存在判断完成。
### 2.1 原始数据与索引

| 文件／目录 | 内容与用途 |
| --- | --- |
| `usb.bin` | 进入本次录制的原始 USB 接收字节，包括传感器帧、收到的控制 ACK，以及无法解码／CRC 错误／首尾不足完整帧的字节。可用于重新解码和审计。 |
| `usb_batches.csv` | 每次非空串口读取一行，记录原始字节偏移、长度、电脑读取起止时间及解码帧数；没有解出完整帧的批次也保留。 |
| `imu.csv` | 每条 CRC 有效的六轴合并数据帧一行，包含原始六轴、H743 时间、接收时间、序号、质量标志、物理量及姿态；不按相机帧率降采样，不按姿态有效性删除六轴行。 |
| `imu_image.csv`、`association.json` | 保存收尾生成的全量 IMU 主表、可为空的图片关联、同帧姿态与时间；规则与数量见第 3 节。 |
| `frames.csv` | 每个已提交双目帧一行，记录媒体时间、电脑读取区间、统一时间、原拼接帧尺寸和左右目相对路径。 |
| `cam0/`、`cam1/` | 输入拼接图的左、右半幅，保存为彩色无损 PNG。文件名为九位补零 `frame_id`，例如同一双目帧的 `cam0/000000000.png` 与 `cam1/000000000.png`。默认每半幅 1280×720，实际以配置及已保存图像为准。 |

**原始录制的 `imu.csv` 不包含图片关联栏**，IMU 与图片分别保存，通过各自索引保留时间信息。你要求的“六轴 → 图片信息 → 四元数／欧拉角”合并布局在同一目录内直接保存的 `imu_image.csv` 中提供，无需导出。

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
| `verification.json` | 单独运行 `verify` 成功后的审计报告，包括 USB 重解码、CSV／双目索引核验、文件 SHA256 等。保存收尾不会自动生成此文件。 |

## 3. 保存主表 `imu_image.csv`

该表直接位于 `raw/` 内，保存完成即可使用，不需要另一次导出。

| 分组顺序 | 列 |
| --- | --- |
| ① 原始六轴 | `ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw` |
| ② 图片信息 | `image_frame_id, image_timestamp_ns, image_minus_imu_ns, cam0, cam1` |
| ③ 四元数／欧拉角 | `qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg, attitude_valid` |
| ④ 时间与补充信息 | 其余全部源 IMU 字段，保持其相对顺序，包括编号、统一时间、H743 时间、序号、质量及物理量。旧会话缺少统一时间列时依据原始 USB 批次接收时间补充。 |

- **每条 CRC 有效 IMU 合并帧一行，全部保留**，不按图片频率降采样、不只保留匹配行，也不按姿态有效性删除六轴。
- **每行以该条六轴自己的时间为准**，不改为图片时间。`unified_timestamp_ns` 为 IMU 所属 USB 批次的电脑读取结束时间；H743 `timestamp_us` 与整数乘 1000 的 `timestamp_ns` 原样保留。同批多行可有相同统一时间，不插值或人为递增。
- **无对应图片：五个图片字段全部留空**，不是 0；有效姿态、六轴与时间不受影响。
- **无有效姿态：七个四元数／欧拉角数值字段留空，`attitude_valid=0`**；图片匹配与六轴不受影响。当前 v2 录制也有这些空列；非常旧的源表本来没有姿态列时，不凭空补算或伪造。
- 图片与姿态都缺失时，两组分别留空，仍保留完整六轴行。
- 四元数与欧拉角来自同条板端 IMU 帧，不由图片解算、重采样或插值得到。
- `image_timestamp_ns` 为图像电脑读取结束时间；`image_minus_imu_ns` 为图像减去 IMU 接收时间的有符号纳秒差。
- `cam0/cam1` 指向原始彩色 PNG，例如 `cam0/000000000.png`，路径相对于 `raw/`，不是外层总会话目录。

### 3.1 关联规则与全部图像保留

保持原规则：按相机帧顺序，在 **20 ms** 接收时间容差内选择最近的尚未匹配 IMU，一对一；等距时选较早 IMU，同批同时间选较早 `sample_id`。不会把同一张图片重复填满所有高频 IMU 行。该规则是顺序贪心，不是全局最优，**容差不是曝光同步精度**。

未匹配到 IMU 的图像仍保留在 `frames.csv` 与两目目录，不凭空创建无 IMU 的关联表行。`imu.csv`、`frames.csv`、`usb.bin` 与原始彩色 PNG 不因关联而改写；不生成灰度图或重复图片副本。

### 3.2 `association.json`

记录表名、时间方案、IMU 时间来源、原始设备时间保留方式、关联规则／容差、全部 IMU 行数、匹配／未匹配数量、图片数／未匹配图片数、空值与同步限制。

`column_groups` 的四组键为 `raw_six_axis`、`image`、`attitude`、`provenance`，值为各组实际列名。`provenance` 在这里是追溯字段分组，不是复制目录。

关联表写入 `imu_image.csv.tmp`、flush/fsync 后替换正式文件；元数据也使用临时文件替换。生成失败不会标记 `finalized=true`，已有正式关联表不会被部分 CSV 替换，但其内容可能属于上一次成功重建，不能忽略当前失败状态。
## 4. 数据之间如何对应

```text
原始 USB 字节 usb.bin
    └─ usb_batches.csv：batch_id + 字节范围 + 电脑读取区间
        └─ imu.csv：sample_id + batch_id + 六轴 + 板端姿态 + 时间

双目彩色 PNG cam0/、cam1/
    └─ frames.csv：frame_id + 两目路径 + 媒体时间 + 电脑读取区间

完成录制并保存收尾（只处理已有数据，不重新采集）
    ├─ aligned_frames.csv + clock_models.json + quality.json
    └─ imu_image.csv + association.json：每条 IMU + 可空图片关联 + 同帧姿态
```

IMU 与相机并行采集，不是“先六轴、再拍图片、最后由图片算姿态”。频率不同，关联表中大量空图片栏是正常的一对一结果。

## 5. 校验、异常与归档建议

- **保留整个总会话文件夹及其 `raw/` 内容。** 单独 CSV 不包含图片像素、USB 字节与质量信息，不能替代完整会话。
- `session.json` 的 `finalized=false` 或缺失表示保存最终化未成功，不能当成完成数据。即使 `finalized=true`，`stopped_error` 或质量报告中的故障仍表示采集可能不完整。
- `failure.json`、队列溢出、未保存计数等需结合检查。双目成功写入后才提交图像索引，故障可能遗留未索引 PNG；不要按文件数量自行补造索引。
- `.json.tmp`、`imu_image.csv.tmp` 不是正式结果，异常中断后可能残留。
- CSV／原始字节关闭时 flush+fsync，元数据和关联表用临时文件替换，但**不保证断电事务原子性**；不要强杀仍在保存的程序。
- 默认保存收尾不逐张解码审计图片；需要时独立运行 `verify --decode-all`。保存完成不等于已完成全量离线审计。
- 统一纳秒时间是电脑单调时钟，不是 UTC；不能将不同电脑／不同系统启动周期的数据直接当同一绝对纪元合并。同批时间重复不表示 MCU 重复采样。
- 不生成虚假内参、外参、标定或同步精度保证。波形历史仍是内存数据，不是录制文件回放。

可在 `viewer` 目录运行以下离线命令，替换示例会话名：

```powershell
# 原始 USB、CSV、双目索引等校验；成功后 raw/ 内生成 verification.json
.\.venv\Scripts\python.exe -m imu_viewer verify .\recordings\session_20261120_140000_123456

# 如需检查每一张左右目图片是否可解码
.\.venv\Scripts\python.exe -m imu_viewer verify .\recordings\session_20261120_140000_123456 --decode-all

# 可选：重建已停止会话的时间索引及 IMU 关联表（不处理仍在录制的会话）
.\.venv\Scripts\python.exe -m imu_viewer finalize .\recordings\session_20261120_140000_123456
```

`finalize` 不是丢失数据恢复工具，无法补回未写入字节或图像；中断且仍标记 `recording` 的会话不直接最终化。旧 schema 1 会话保留原时间模型语义，重建关联表仍依据已保存的电脑接收时间，不用新时间覆盖板端原始值，不补算缺失姿态。详细界面与时间说明见 [README](README.md)。
