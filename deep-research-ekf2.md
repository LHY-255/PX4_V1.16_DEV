# PX4 v1.16 EKF2 模块深度研究：源码工作流、核心调参与典型异常排查

## 研究范围与核心结论

本报告严格基于两类最高优先级资料：PX4 官方文档（docs.px4.io 的 v1.16 分支）与 PX4 Autopilot GitHub 仓库 **v1.16.0 tag** 下 `src/modules/ekf2` 的源码（含 CMakeLists、Kconfig、模块实现与参数定义 YAML）。在“典型场景调参逻辑”部分，补充引用 Discuss PX4（次优先级）中与 v1.16 相关或可迁移的工程经验贴。 citeturn15view0turn18view0turn25view0turn29view0turn35view0

结论先行（便于你后续做传感器融合精准调参/剪裁）：

1. **EKF2 的主循环是“IMU 驱动”**：每次 IMU 样本到来，先 `_ekf.setIMUData()`，并**立即发布姿态**（利用 output predictor 的四元数），随后再拉取并注入其他传感器样本，最后 `_ekf.update()`，成功后发布 local/global/odometry 等输出与诊断话题。 citeturn18view0
2. **各传感器测量不在“当前时刻”直接融合**：EKF2 在“延迟的融合时域（fusion time horizon）”运行，所有传感器数据以 FIFO 缓冲，按各自 `EKF2_*_DELAY` 补偿后在正确时刻融合；`EKF2_DELAY_MAX` 决定缓冲区时域长度；融合后再用互补滤波把状态推送到当前，时间常数由 `EKF2_TAU_VEL / EKF2_TAU_POS` 控制。 citeturn16view1turn16view2turn34view0
3. **v1.15 起 `EKF2_AID_MASK` 已移除**，配置多传感器融合改为按传感器族分组的 `EKF2_*_CTRL`（如 `EKF2_GPS_CTRL`、`EKF2_EV_CTRL`），这对你做固件剪裁/定制参数界面非常关键。 citeturn35view0turn33view1turn33view3
4. 对飞行稳定性影响最大的调参“杠杆”基本集中在四类：**杆臂/安装误差、噪声与置信、创新门限、时间延迟**。这些参数在 v1.16.0 已由 `src/modules/ekf2/*.yaml` 统一定义（QGC 侧显示/隐藏也会受编译剪裁影响）。 citeturn34view0turn33view1turn33view2turn33view3turn33view4

## 源码结构与剪裁入口：src/modules/ekf2 的“工程分层”

在 PX4 v1.16.0 中，EKF2 模块可把握为三层（都在 `src/modules/ekf2` 目录中）：

第一层是 **模块/线程与 uORB 接入层**（你要读懂数据流向的核心）。关键文件是 `src/modules/ekf2/EKF2.cpp`：负责订阅 uORB、组装各类 sample、调用 `_ekf.set*Data()`、执行 `_ekf.update()` 并发布输出/诊断话题。其代码清晰显示：IMU 更新触发主循环；各传感器样本在同一循环内被拉取并注入 EKF；更新成功后发布 local/global/odometry 与 innovations 等。 citeturn18view0turn24view0turn25view0

第二层是 **EKF 核心算法与各观测源融合模块**（“预测 + 观测更新”的主体实现）。从 `src/modules/ekf2/CMakeLists.txt` 可以直接看到 EKF 核心源文件集合（如 `EKF/control.cpp`, `EKF/position_fusion.cpp`, `EKF/velocity_fusion.cpp`, `EKF/yaw_fusion.cpp` 等），以及按 `CONFIG_EKF2_*` 条件编译的“aid_sources/xxx/*_control.cpp / *_fusion.cpp”。这对你做“剪裁”和“只保留某些传感器融合路径”非常直接：关掉相应 Kconfig，就不会编译对应源码与参数 YAML。 citeturn5view1

第三层是 **编译期特性开关（Kconfig）与参数元数据（YAML）**。
`src/modules/ekf2/Kconfig` 逐项列出 EKF2 能否编译 GNSS、磁力计、外部视觉、光流、测距、风估计等支持（`EKF2_GNSS`/`EKF2_MAGNETOMETER`/`EKF2_EXTERNAL_VISION`/`EKF2_OPTICAL_FLOW`/`EKF2_RANGE_FINDER`/`EKF2_WIND` …）。在 Cortex‑M7（类似 FMU‑v6x）做固件剪裁时，这就是你降低 Flash/RAM/CPU 占用的“第一道闸门”。 citeturn29view0turn5view1

## EKF2 核心工作流解析：从 uORB 订阅到预测、更新与输出

下面用“源码级数据流”把 EKF2 跑通（基于 PX4 v1.16.0 tag），并在关键处标注核心代码路径。

### 传感器输入与 sample 注入：Update*Sample() 的真实订阅话题

`src/modules/ekf2/EKF2.cpp` 的主循环在拿到 IMU 更新后，会依次调用若干 `Update*Sample()`，每个函数都从特定 uORB 话题取数据并构造 EKF 内部的 sample 结构体，然后调用 `_ekf.set*Data()` 注入“融合缓冲队列”。这一点是理解“数据从哪里来、以何种时间戳进入 EKF”的关键。 citeturn18view0turn25view0turn24view0

核心输入链路（按源码出现顺序）如下：

1) **IMU（预测驱动源）**
EKF2 支持单实例与多实例/选择器两种 IMU 接入：
- 多实例模式：订阅 `vehicle_imu`（可按 instance 区分），将 `delta_angle/delta_velocity` 与 `timestamp_sample` 组装为 `imuSample`。 citeturn18view0turn23view3
- 非多实例模式：订阅 `sensor_combined`，用 `gyro_integral_dt/accelerometer_integral_dt` 计算积分量同样形成 `imuSample`。 citeturn18view0

随后每次 IMU 更新都会执行：`_ekf.setIMUData(imu_sample_new);` 并用于状态推进（预测）。 citeturn18view0

2) **气压计/空速相关（高度与空密度）**
`UpdateBaroSample()` 订阅 `vehicle_air_data`（`vehicle_air_data_s`），把 `baro_alt_meter` 作为 `baroSample` 注入 `_ekf.setBaroData()`，并把 `rho`（空气密度）传给 EKF（`_ekf.set_air_density(airdata.rho)`）。同时监视 `baro_device_id` 与 `calibration_count` 变化触发 reset。 citeturn25view0

3) **GNSS（定位/速度/航向）**
`UpdateGpsSample()` 使用 `_vehicle_gps_position_sub` 更新 `sensor_gps_s`（源码注释“EKF GPS message”，变量名虽叫 `vehicle_gps_position`，但类型是 `sensor_gps_s`），构造 `gnssSample` 并调用 `_ekf.setGpsData()` 注入。其字段覆盖 `lat/lon/alt(AMSL)/vel_ned/hacc/vacc/sacc/fix_type/nsats/pdop`，并包含 dual‑antenna heading 的 `yaw/yaw_acc/yaw_offset` 与 spoofing 标志。 citeturn24view0

4) **磁力计**
`UpdateMagSample()` 订阅 `vehicle_magnetometer`（`vehicle_magnetometer_s`），将 `magnetometer_ga` 注入 `_ekf.setMagData(magSample{timestamp_sample, field, reset})`，同样以 `device_id` 与 `calibration_count` 变化触发 reset。 citeturn24view0

5) **外部视觉（VIO/MoCap/SLAM）**
`UpdateExtVisionSample()` 订阅 `vehicle_odometry`（`vehicle_odometry_s`），根据 `pose_frame/velocity_frame` 判断坐标系合法性，并对 position/velocity/orientation 做有限性与四元数归一性检查（含“非零、元素不超过 1、范数接近 1”）。噪声/方差来源支持两种模式：
- `EKF2_EV_NOISE_MD=0`：优先用消息内 variance，但以参数噪声为下界；
- `EKF2_EV_NOISE_MD=1`：直接用参数生成观测方差。
此外源码明确写到：EV 使用外部计算机的时间戳，“当使用 MAVROS 时钟同步”。这与你做延迟补偿与时间同步调参强相关。 citeturn25view0turn33view3

6) **光流**
`UpdateFlowSample()` 订阅 `vehicle_optical_flow`（`vehicle_optical_flow_s`），用 `integration_timespan_us` 计算 dt，并把像素流与角增量换算为“rate”（同时注明 EKF 使用与传感器相反的符号约定）。它将时间戳校正到积分区间中点：`time_us = timestamp_sample - integration_timespan_us/2`。并把传感器上报的 `max_flow_rate/min_ground_distance/max_ground_distance` 传给 EKF 作为限制。 citeturn25view0

7) **测距（Range Finder / Distance Sensor）**
`UpdateRangeSample()` 使用 `distance_sensor` uORB group（`distance_sensor_s`），会在第一次选择时扫描多个实例，优先选择 **朝下** 且“最近 0.1s 内更新”的传感器；选定后持续读取并注入 `_ekf.setRangeData(rangeSample{timestamp, current_distance, signal_quality})`，同时把 `min_distance/max_distance` 设置为 EKF rangefinder limits。若超过 1s 未更新则强制重新选择。 citeturn24view0

8) **辅助速度（AuxVel，典型来自 landing_target）**
`UpdateAuxVelSample()` 订阅 `landing_target_pose`（`landing_target_pose_s`），当目标静止且 `rel_vel_valid` 时，将相对速度转为无人机速度并注入 `_ekf.setAuxVelData()`。这在“无 GNSS、低空、视觉/光流与目标跟踪组合”时可能成为速度约束源。 citeturn25view0

9) **系统状态标志（决定融合模式的逻辑输入）**
`UpdateSystemFlagsSample()` 读取 `vehicle_status`、`vehicle_land_detected`、`launch_detection_status` 等，形成 `systemFlagUpdate` 注入 EKF：包括 `in_air/at_rest/gnd_effect/is_fixed_wing/constant_pos` 等关键条件。这些标志会影响 EKF 内部控制逻辑（比如起飞前后、地效区、固定翼模式等）。 citeturn24view0

### 预测与观测更新：延迟融合时域 + FIFO 缓冲的工程含义

官方 v1.16 文档明确：EKF 在**延迟的融合时间范围**上运行；每个传感器数据都 FIFO 缓冲，EKF 从缓冲区检索并在正确时间使用；各传感器延迟补偿由 `EKF2_*_DELAY` 控制；缓冲区的“融合时域”长度由 `EKF2_DELAY_MAX` 决定，且它应不小于所有 `EKF2_*_DELAY` 的最大值。 citeturn16view1turn16view2turn34view0

从工程调参角度，这意味着：

- 你在 QGC 调 `EKF2_GPS_DELAY / EKF2_EV_DELAY / EKF2_OF_DELAY / ...` 本质是在改变“观测被放入滤波器融合队列后、被取出用于观测更新”的对齐时间。调错会直接表现为：位置/速度/姿态在动态机动时出现相位滞后、超调或创新（innovation）异常增大。 citeturn16view1turn33view1turn33view3turn33view6
- EKF2 通过互补滤波把“融合时域的状态”推到当前时间输出，`EKF2_TAU_VEL / EKF2_TAU_POS` 决定输出平滑与滞后之间的折中（跟控制回路体感非常强相关）。 citeturn16view1turn34view0

### 输出发布：哪些 uORB 话题是控制回路与日志分析的抓手

从源码主循环可见：当 `_ekf.update()` 成功后，EKF2 会发布本地位置、里程计、全局位置、传感器 bias、风估计（若编译/启用）以及一系列状态与诊断消息；当 `EKF2_LOG_VERBOSE` 打开时，还会额外发布 aid source 状态、innovations、test ratios、innovation variances、states 等用于日志诊断的 uORB 消息。 citeturn18view0turn34view0

官方 v1.16 文档也给出了“最常用的输出话题”定位方式：姿态在 `VehicleAttitude`，本地位置在 `VehicleLocalPosition`，全局位置在 `VehicleGlobalPosition`，风在 `Wind`；同时指出 EKF 关键诊断数据主要在 `EstimatorInnovations` 与 `EstimatorStatus`（并提供 `Tools/ecl_ekf` 下的日志处理脚本入口）。 citeturn15view0turn16view3

一个对“物理杆臂/安装误差”特别关键但经常被忽略的点：文档说明**位置与速度状态在输出到控制回路前，会根据 IMU 相对机体坐标系的偏移量进行修正**，此偏移由 `EKF2_IMU_POS_X/Y/Z` 设置。也就是说 IMU 杆臂不是“纯日志美观参数”，而会真实影响控制回路所用的状态量。 citeturn16view1turn34view0

## 工程调参核心参数表：按稳定性影响维度整理

下表只摘取“直接影响飞行稳定性/体感”的核心参数（PX4 v1.16.0），并按你指定的四个维度组织。每个参数均给出**参数元数据来源文件路径**（最高优先级），便于你在固件剪裁后同步维护参数集合。

> 说明：表中“默认值”来自 v1.16.0 `src/modules/ekf2/*.yaml`；如你在剪裁时关闭某个 `CONFIG_EKF2_*`，对应 YAML 可能不会被 CMakeLists 纳入模块参数集（参数将不出现在固件中）。 citeturn5view1turn29view0turn33view1

| 维度 | 核心参数（v1.16.0） | 默认值 | 工程含义（摘要） | 调参抓手（面向稳定性） | 关键来源（代码/文档路径） |
|---|---|---:|---|---|---|
| Lever Arms & Offsets | `EKF2_IMU_POS_X/Y/Z` | 0/0/0 m | IMU 相对质心的机体系位置；并用于输出到控制回路前的状态修正 | 杆臂填错常见表现：动态机动时姿态/速度耦合异常、位置控制“甩尾/延迟感” | `src/modules/ekf2/module.yaml`（参数定义）；文档说明输出会按 IMU 偏移修正 citeturn34view0turn16view1 |
| Lever Arms & Offsets | `EKF2_GPS_POS_X/Y/Z` | 0/0/0 m | GNSS 天线相对质心位置 | RTK/双天线时尤关键：杆臂错会造成转弯/加减速时位置与航向耦合误差 | `src/modules/ekf2/params_gnss.yaml` citeturn9view1 |
| Lever Arms & Offsets | `EKF2_EV_POS_X/Y/Z` | 0/0/0 m | 外部视觉传感器（VIO/MoCap）焦点相对质心位置 | 室内/拒止环境强相关：杆臂错常导致“位姿对得上但控制发散/漂移” | `src/modules/ekf2/params_external_vision.yaml` citeturn11view0 |
| Lever Arms & Offsets | `EKF2_OF_POS_X/Y/Z` | 0/0/0 m | 光流焦点相对质心位置 | 低空定点/贴地飞行：错位会让横向速度/位置估计出现系统性偏差 | `src/modules/ekf2/params_optical_flow.yaml` citeturn10view2 |
| Lever Arms & Offsets | `EKF2_RNG_POS_X/Y/Z` | 0/0/0 m | 测距传感器原点相对质心位置 | 起降高度控制：错位会放大俯仰滚转时的高度误差（尤其非垂直安装） | `src/modules/ekf2/params_range_finder.yaml` citeturn11view1 |
| Lever Arms & Offsets | `EKF2_RNG_PITCH` | 0 rad | 测距传感器俯仰安装偏差 | 常用于补偿“测距不是正对地面”导致的高度系统误差 | `src/modules/ekf2/params_range_finder.yaml` citeturn11view1 |
| Lever Arms & Offsets | `EKF2_GPS_YAW_OFF` | 0 deg | 双天线 GNSS 航向的安装偏置 | 双天线方向装反/偏置会直接反映为 yaw 系统误差与位置模式画圈 | `src/modules/ekf2/params_gnss.yaml` citeturn9view1 |
| Noise & Variance | `EKF2_GYR_NOISE` | 0.015 rad/s | 陀螺噪声（协方差预测） | 过小：滤波器过度信 IMU，外部观测被“当异常”；过大：姿态更“漂”，控制更松散 | `src/modules/ekf2/module.yaml` citeturn34view0 |
| Noise & Variance | `EKF2_ACC_NOISE` | 0.35 m/s² | 加速度计噪声（协方差预测） | 振动大/桨噪明显时常需适度增大，否则高度/速度创新容易异常 | `src/modules/ekf2/module.yaml` citeturn34view0 |
| Noise & Variance | `EKF2_GYR_B_NOISE` | 0.001 rad/s² | 陀螺 bias 过程噪声 | 过小：bias 跟踪慢、温漂难跟；过大：bias 抖动、姿态噪声上升 | `src/modules/ekf2/params_gyro_bias.yaml` citeturn12view0 |
| Noise & Variance | `EKF2_ACC_B_NOISE` | 0.003 m/s³ | 加速度计 bias 过程噪声 | 与振动/温漂/机动强相关，直接影响速度/高度长期漂移与恢复能力 | `src/modules/ekf2/params_accel_bias.yaml` citeturn12view1 |
| Noise & Variance | `EKF2_GPS_P_NOISE` | 0.5 m | GNSS 位置测量噪声 | GNSS 干扰/多路径时可适当增大以降低 GPS 权重（但不要用它替代 GPS 健康检查） | `src/modules/ekf2/params_gnss.yaml` citeturn9view1 |
| Noise & Variance | `EKF2_GPS_V_NOISE` | 0.3 m/s | GNSS 速度测量噪声 | 影响“无磁/无视觉时 yaw 对齐与漂移抑制”（GNSS 速度约束很关键） | `src/modules/ekf2/params_gnss.yaml` citeturn9view1 |
| Noise & Variance | `EKF2_BARO_NOISE` | 3.5 m | 气压高度测量噪声 | 过小：易把气压瞬态（桨下洗/动压）当真导致高度抖；过大：高度漂移、响应慢 | `src/modules/ekf2/params_barometer.yaml` citeturn9view2 |
| Noise & Variance | `EKF2_MAG_NOISE` | 0.05 gauss | 磁力计三轴融合测量噪声 | 磁干扰环境可适度增大以降低磁观测权重；更根本是启用检查/隔离干扰源 | `src/modules/ekf2/params_magnetometer.yaml` citeturn10view0 |
| Noise & Variance | `EKF2_EVP_NOISE / EKF2_EVV_NOISE / EKF2_EVA_NOISE` | 0.1 m / 0.1 m/s / 0.1 rad | 外部视觉位置/速度/姿态测量噪声（用于下界或直接指定） | 当 EV 协方差不可用或不可信时，用 `EKF2_EV_NOISE_MD` 切到“参数噪声模式”再调这三项 | 参数：`params_external_vision.yaml`；以及 v1.16 文档解释 EV 协方差与噪声模式 citeturn11view0turn15view0 |
| Innovations Gates | `EKF2_GPS_P_GATE / EKF2_GPS_V_GATE` | 5 / 5 SD | GNSS 位置/速度创新门限（标准差倍数） | 过小：GPS 常被拒导致位置模式不可用；过大：易接受坏 GPS 造成“瞬移/画圈” | `src/modules/ekf2/params_gnss.yaml` citeturn9view1 |
| Innovations Gates | `EKF2_BARO_GATE` | 5 SD | 气压/高度融合创新门限 | 高动压/桨下洗环境可适度收紧或配合动压补偿（PCOEF）而非单靠放大 gate | `src/modules/ekf2/params_barometer.yaml` citeturn9view2 |
| Innovations Gates | `EKF2_MAG_GATE` | 3 SD | 磁三轴融合创新门限 | 磁干扰时：更推荐配合 `EKF2_MAG_CHECK` 与 `EKF2_MAG_TYPE`，gate 只是在“拒不拒”的最后一道门 | `src/modules/ekf2/params_magnetometer.yaml` citeturn10view0 |
| Innovations Gates | `EKF2_HDG_GATE` | 2.6 SD | 航向（磁航向）融合 gate | 室内/弱磁环境易触发 yaw 问题；可结合“无磁靠 GNSS 速度对齐”策略评估 | `src/modules/ekf2/module.yaml`；磁融合策略说明见 `EKF2_MAG_TYPE` 描述 citeturn34view0turn33view5 |
| Innovations Gates | `EKF2_EVP_GATE / EKF2_EVV_GATE` | 5 / 3 SD | 外部视觉位置/速度融合 gate | EV 偶发跳变/重定位时 gate 常是第一排查点；但根因多在时间戳/坐标系/协方差 | `src/modules/ekf2/params_external_vision.yaml` citeturn11view0 |
| Innovations Gates | `EKF2_OF_GATE` | 3 SD | 光流融合 gate | 室内光照差/纹理少时易出现创新异常；不要盲目放大 gate，先看质量阈值与噪声映射 | `src/modules/ekf2/params_optical_flow.yaml` citeturn10view2 |
| Innovations Gates | `EKF2_RNG_GATE / EKF2_RNG_A_IGATE / EKF2_RNG_K_GATE` | 5 / 1 / 1 SD | 测距融合 gate、条件测距启用的稳定性 gate、运动学一致性 gate | 低空起降：先调 `EKF2_RNG_NOISE` 与 `EKF2_RNG_SFE` 再动 `EKF2_RNG_K_GATE`（参数描述也明确建议） | `src/modules/ekf2/params_range_finder.yaml` citeturn11view1 |
| Time Delays | `EKF2_GPS_DELAY` | 110 ms | GNSS 相对 IMU 的测量延迟 | GNSS/RTK 链路带宽与驱动延迟不同，调错会导致转弯/加速时位置滞后与创新增大 | `src/modules/ekf2/params_gnss.yaml`；延迟机制官方说明 citeturn9view1turn16view1 |
| Time Delays | `EKF2_EV_DELAY` | 0 ms | 外部视觉相对 IMU 延迟 | 视觉链路常是最大延迟源之一；同时要确保 `EKF2_DELAY_MAX` ≥ 本值 | `src/modules/ekf2/params_external_vision.yaml`；延迟机制官方说明 citeturn11view0turn16view1 |
| Time Delays | `EKF2_OF_DELAY` | 20 ms | 光流相对 IMU 延迟 | 光流本身是积分量，参数描述指出假设“积分区间尾沿时间戳”，调时要结合传感器实现 | `src/modules/ekf2/params_optical_flow.yaml` citeturn10view2 |
| Time Delays | `EKF2_MAG_DELAY / EKF2_BARO_DELAY / EKF2_RNG_DELAY / EKF2_ASP_DELAY` | 0 / 0 / 5 / 100 ms | 磁/气压/测距/空速相对 IMU 延迟 | 多源融合时，谁延迟最大、谁最不稳，往往决定 `EKF2_DELAY_MAX` 与整体动态表现 | 参数 YAML：mag/baro/rng/airspeed；延迟机制官方说明 citeturn10view0turn9view2turn11view1turn11view2turn16view1 |
| Time Delays | `EKF2_DELAY_MAX` | 200 ms | 融合时域缓冲区最大延迟（必须覆盖最大 `EKF2_*_DELAY`） | 缓冲区过短：延迟补偿失效；过长：输出预测误差可能变大且增加内存/延迟感 | `src/modules/ekf2/module.yaml`；v1.16 文档说明 citeturn34view0turn16view1 |

## 典型场景调参逻辑：常见异常现象到“可执行”的排查路径

以下 3 个场景都选自 Discuss PX4 常见工程问题（并尽量贴近 v1.16.x）；每个场景给出“先看什么日志/状态 → 再动哪些参数 → 为什么”。

### 现象一：Position 模式“画圈/Toilet Bowl Effect”，常与航向（Yaw）/磁干扰/GNSS 异常耦合

社区里对“toilet bowling/画圈”的描述高度一致：一切在 Altitude/高度模式看似正常，但一切到 Position（依赖导航解）就开始绕圈扩大，且常被怀疑与 GPS、罗盘靠近大电流线束或电源有关。 citeturn30view1turn30view0

建议排查逻辑（强烈建议按顺序，避免“靠加大 gate 掩盖根因”）：

1) **先确认问题是 yaw 还是 position 本身**：很多“绕圈”本质是 yaw 估计错，导致位置控制在错误航向坐标下闭环。社区回复也直接指出“toilet bowling due to incorrect yaw estimates”。 citeturn30view2
2) **对磁力计做 A/B 测试**：v1.16 的 `EKF2_MAG_TYPE` 支持直接设为 `None`（不使用磁力计），并明确说明：若无外部 yaw 源，也可以靠“起飞后水平运动 + GNSS 速度测量”完成 yaw 对齐。这给了你一个很实用的验证手段：**禁用磁后绕圈显著改善 → 根因多半在磁干扰/磁融合策略**。 citeturn33view5turn33view1
3) **磁相关参数如何动**：
   - 优先启用/收紧磁有效性检查而不是盲目放大 `MAG_GATE`：`EKF2_MAG_CHECK` 允许选择磁场强度/倾角检查并可等待 WMM（世界磁模型）以提供理论强度与倾角；阈值由 `EKF2_MAG_CHK_STR / EKF2_MAG_CHK_INC` 控制。 citeturn10view0
   - 若确实需要提高抗干扰，才考虑适度增大 `EKF2_MAG_NOISE` 或 `EKF2_MAG_GATE`，但要清楚这会降低磁对 yaw 的约束强度，可能增加长时间漂移风险。 citeturn10view0
4) **GNSS 侧不要只调噪声/门限，先看“健康检查”**：官方文档指出，EKF 接纳 GNSS 需要满足一段时间内的最小性能要求（由 `EKF2_REQ_GPS_H` 定义），阈值由 `EKF2_REQ_*` 系列参数给出，各检查可由 `EKF2_GPS_CHECK` 开关控制。GNSS 干扰/多路径导致绕圈时，**先保证 check 机制能把坏 GNSS 拦在融合之外**。 citeturn15view0turn9view1
5) **最后才是“权重与 gate”层面的工程权衡**：
   - `EKF2_GPS_P_NOISE / EKF2_GPS_V_NOISE` 会影响 GNSS 观测对解的拉动强度；
   - `EKF2_GPS_P_GATE / EKF2_GPS_V_GATE` 决定创新一致性测试的拒绝敏感度。 citeturn33view1
6) **社区经验的“应急式调整”要谨慎使用**：有人报告通过增大 `EKF2_GYRO_NOISE` 能减小绕圈半径（本质是让滤波更承认预测不确定性、可能更愿意接受外部观测或更快膨胀协方差）。这可以作为临时验证，但不建议作为最终根治方案（根因仍应回到磁干扰/时间戳/传感器质量）。 citeturn30view2turn34view0

### 现象二：高度“掉高/刹车下沉/起降突跳”，典型根因是气压动压误差、地效/桨下洗、测距融合策略不当

一个很高频的工程现象是：高速或刹车后高度突然下沉几米，随后又恢复；社区案例直接指出“barometer is affected significantly when stopping”，并引用 PX4 文档的“静压位置误差修正（Static Pressure Position Error）”。 citeturn32view1turn9view2

建议排查逻辑：

1) **把“高度源”与“误差类型”分开看**：如果高度参考是 baro，那么很多“动态高度掉高”其实不是 EKF 数学错，而是 baro 在机体气流场中产生了系统性误差。对应地，v1.16 提供了静压误差模型系数 `EKF2_PCOEF_XP/XN/YP/YN/Z` 与 `EKF2_ASPD_MAX`。参数描述明确：这些系数是“静压误差与动压的比例”，并提示如果前飞导致 baro 高度上升则系数应为负。 citeturn9view2turn32view1
2) **高度创新 gate/噪声不应代替动压补偿**：`EKF2_BARO_NOISE` 与 `EKF2_BARO_GATE` 能影响 baro 融合的“相信程度/拒绝程度”，但对“随速度变化的系统误差”往往治标不治本。更工程化的路径是：先通过静压补偿脚本/模型确定 `PCOEF`，再微调 noise/gate。 citeturn9view2turn32view1
3) **起降阶段的桨下洗/地效：优先用“条件测距辅助”而非硬怼气压**：`EKF2_RNG_CTRL=1`（conditional mode）被参数长描述明确定位为：在低速（< `EKF2_RNG_A_VMAX`）且低高度（< `EKF2_RNG_A_HMAX`）时启用测距，以应对“rotor wash 对 baro 的干扰会腐蚀 EKF 状态”。这几乎就是为“起飞/降落高度抖动、掉高”定制的工程开关。 citeturn33view4turn11view1
4) **如果你怀疑“振动导致的速度/高度漂移”**：社区在 2026 年的讨论里提出过“高振动是否可能让垂直速度持续偏差而创新比仍健康？”并有跟帖指出振动是唯一明显问题、以及重新启用 EV 速度融合后明显改善。对工程调参而言，这意味着：
   - 机械侧先把振动压到可接受范围；
   - 估计侧再考虑 `EKF2_ACC_NOISE`、加速度计 bias 学习相关参数（如 `EKF2_ABL_ACCLIM` 等）与是否需要额外速度观测（EV velocity / GNSS velocity）。 citeturn32view0turn34view0turn33view9

### 现象三：外部视觉漂移/不对齐/被忽略，常见根因是时间同步、坐标系/帧定义、协方差/质量字段与门限

v1.16 环境下，外部视觉问题在水下/室内项目中非常典型：有人在 v1.16.0 上遇到 `estimator_status_flags` 中 EV 长期为 false、并伴随 time sync 警告；也有人在 Qualisys 融合中出现“初始对齐但随后漂移并偶尔恢复、甚至彻底不一致”。 citeturn32view3turn30view3

建议排查逻辑（强烈建议以“是否满足源码的输入合法性检查”为第一原则）：

1) **先用源码标准检查你的 `vehicle_odometry` 输入是否会被丢弃**：`UpdateExtVisionSample()` 对 position/velocity/orientation 都做了严格合法性判断：
   - 必须是可识别的 `pose_frame/velocity_frame`；
   - position/velocity 必须 finite；
   - 四元数必须非零、元素不超界、范数接近 1 且 finite，然后会被 normalize。
   任一环节不满足，就可能导致 EV 数据根本没进入 `_ekf.setExtVisionData()`。 citeturn25view0
2) **时间同步与延迟补偿是“外部视觉稳定性”的生命线**：源码明确写道 EV 使用外部计算机时间戳，并假设在 MAVROS 场景下时钟同步；另一方面，官方文档强调 EKF 在延迟融合时域运行，延迟由 `EKF2_*_DELAY` 控制、缓冲长度由 `EKF2_DELAY_MAX` 控制。工程上你需要同时满足：
   - 外部里程计时间戳单调且与 FC 时间基准一致（或至少延迟可建模）；
   - `EKF2_EV_DELAY` 设定正确且 ≤ `EKF2_DELAY_MAX`；
   - 若系统出现“time jump/time sync no longer converged”，需要优先解决时钟同步链路，而不是单纯调 gate。 citeturn25view0turn16view1turn33view3turn32view3
3) **协方差/噪声模式与 gate 的配合**：v1.16 定义了 `EKF2_EV_NOISE_MD`：模式 0 用消息内 variance（参数作下界），模式 1 直接用参数噪声。若你的 VIO/MoCap 侧协方差不可信或缺失，推荐切到模式 1 并调 `EKF2_EVP_NOISE/EKF2_EVV_NOISE/EKF2_EVA_NOISE`；若数据偶发跳变，再通过 `EKF2_EVP_GATE/EKF2_EVV_GATE` 控制拒绝敏感度。 citeturn33view3turn25view0
4) **质量字段与启用门槛**：`EKF2_EV_QMIN` 允许你要求外部视觉质量高于阈值才开始融合，这对“偶发丢跟踪/重定位”的系统很实用。 citeturn33view3

## Cortex‑M7/FMU‑v6x 剪裁与调参联动的注意点

你当前在 Cortex‑M7（类似 FMU‑v6x）做底层固件开发与剪裁时，需要把“估计稳定性”与“编译剪裁”绑定考虑：一旦剪掉某类融合源，不仅 EKF 行为变了，**参数集合、QGC 展示、日志诊断话题**也会随之变化。

关键提醒：

- **剪裁入口一：Kconfig**。`src/modules/ekf2/Kconfig` 提供 GNSS/磁力计/外部视觉/光流/测距/风/多实例等开关；关闭对应项可以显著降低资源占用，但要同步评估你是否还保有足够的观测约束（尤其 yaw 与高度）。 citeturn29view0
- **剪裁入口二：CMakeLists 条件源文件与参数 YAML**。`src/modules/ekf2/CMakeLists.txt` 明确：每关掉一个 `CONFIG_EKF2_*`，不仅 `EKF/aid_sources/...` 的融合源码会被移除，相关 `params_*.yaml` 也不会被加入模块参数集。这会导致 QGC 搜不到参数、日志分析脚本期望的字段减少。 citeturn5view1
- **配置入口变化：`EKF2_AID_MASK` → `EKF2_*_CTRL`**。Discuss 明确指出 `EKF2_AID_MASK` 在 1.15 被移除并由 `EKF2_***_CTRL` 替代；因此你在 v1.16 上做传感器组合切换，应围绕 `EKF2_GPS_CTRL/EKF2_EV_CTRL/EKF2_RNG_CTRL/...` 建立参数模板，而不是复用旧教程。 citeturn35view0turn33view1turn33view3turn33view4

最后补一句“最省时间的调参方法论”，完全来自官方 v1.16 文档路线：把核心分析聚焦在 `EstimatorInnovations`、`EstimatorStatus` 等 EKF 诊断消息，并用 `Tools/ecl_ekf/process_logdata_ekf.py` 批处理生成图与指标，再回到上表四类参数做针对性调节——这比“凭感觉拧参数”稳定得多。 citeturn15view0turn16view3
