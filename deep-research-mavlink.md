# PX4 v1.16 多机集群的 MAVLink 极致压缩方案研究

## 约束与总体压缩思路

你当前的约束是“同一站控同时监控/控制 10–30 架次”，且使用 LR24/LR900（物理链路带宽受限，>5 架次即拥堵）。要在 **固件层**把 MAVLink 链路压到极致，PX4 v1.16 的关键点是：**把“默认启动脚本里对 MAVLink streams 的高频流”彻底裁掉**，然后只保留“位置监控 + 基础状态”所需的最小消息集合，并把每个消息的频率压到可接受的下限。

在 PX4 中，周期性消息由 **stream** 机制发送：启动 mavlink 实例时可指定 `-m <mode>`（决定默认流集合与默认频率），运行后可用 `mavlink stream -s <STREAM> -r <Hz>` 逐条配置 stream 频率（`0`=关闭，`-1`=恢复默认）。PX4 的 MAVLink sender 会在固定循环中发送，并且当综合带宽超过 `mavlink start -r <B/s>` 的限制或物理链路饱和时，会自动按比例降低各流的发送速率；你可以用 `mavlink status` 看 `rate mult` 是否小于 1 来判断是否被限流了。citeturn52view1turn52view0turn52view2

因此工程上最实用的“极致压缩三板斧”是：

1. **模式降档**：把遥测链路（TELEM/LR24/LR900）上的 MAVLink mode 设为 `minimal` 或 `custom`（强烈建议 `custom`，因为它默认不主动推流，便于你做严格白名单）。citeturn52view0turn52view2  
2. **全局发送上限**：用 `mavlink start -r` 把每个实例的最大发送速率（B/s）限制在一个可控范围内（例如每机 1–3KB/s 的量级，30 架次合计就是 30–90KB/s，再结合空口效率与重发策略再往下压）。citeturn52view0turn52view2  
3. **流级白名单**：用 `mavlink stream` 把“必须保留”的消息设成低频，把“高频耗流”的消息全部设为 0（关闭）。citeturn52view1turn52view2  

下面按你要求，结合 PX4 v1.16 的启动脚本与 MAVLink stream 名称，给出可直接落地的裁剪清单与配置方式。

## 启动脚本与默认高耗流流量画像

### rc.mavlink 在 v1.16 的实际落点

你提到“分析 `rc.mavlink` 启动脚本”。在 PX4 v1.16 的 SITL/Posix 启动脚本体系中，MAVLink 启动脚本位于：

- `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`（SITL/Posix）citeturn26view1

该脚本不仅 `mavlink start`，还通过大量 `mavlink stream -s ... -r ...` 明确设置默认流与频率，是你做“裁剪清单”的最可控入口之一。

### px4-rc.mavlink 中的默认 stream 与频率（高耗流重点）

在 `px4-rc.mavlink`（v1.16.0）里，针对 **GCS UDP 链路**默认会启用如下高频 stream（节选重点）：

- 50 Hz：`POSITION_TARGET_LOCAL_NED`、`LOCAL_POSITION_NED`、`GLOBAL_POSITION_INT`、`ATTITUDE`、`ATTITUDE_QUATERNION`、`ATTITUDE_TARGET`、`SERVO_OUTPUT_RAW_0`、`RC_CHANNELS`  
- 10 Hz：`OPTICAL_FLOW_RAD`  

这些流在“单机 + 姿态球 + 调参/调试”场景非常合理，但在“30 机只做位置监控、基础状态”下属于典型的带宽杀手，必须系统性关闭/降频。citeturn26view1

### 多机端口分配逻辑（必须知道，否则 10+ 架容易端口冲突/误连）

`px4-rc.mavlink` 对多实例的 UDP 端口有明确计算方式（这对你在 WSL2+QGC 或使用 mavlink-router 压测非常关键）：

- GCS：本地端口 `18570 + px4_instance`，远端端口固定 `14550`（QGC 默认监听 14550）citeturn26view1turn55view3  
- Offboard：本地端口 `14580 + px4_instance`，远端端口通常为 `14540 + px4_instance`，但 **当实例号 > 9 时远端端口会统一落到 14549**（意味着默认逻辑只“顺序分配”到 14540–14549，超过 10 架会端口复用）。citeturn26view1turn55view1  

PX4 v1.16 文档也明确说明：多机仿真时外部开发者 API 的远端端口会从 `14540` 顺序分配到 `14549`，更多实例会继续使用 `14549`。citeturn55view1turn55view3  

对你“30 架次集群监控”的压测来说，这意味着：

- **如果你只用 GCS 口（14550）做位置监控**：可以让所有实例都发往 14550（靠 sysid 区分），并且各实例本地端口不同即可，通常不会冲突。citeturn26view1turn55view3  
- **如果你还要跑 offboard 口**：默认策略对 10 架以上会复用 14549，你需要主动改脚本或引入路由/代理来消除复用（后文给出压测网络方案建议）。citeturn55view1turn26view1  

## 30 架次仅“位置监控 + 基础状态”的 MAVLink 消息裁剪清单

### 裁剪原则与“必须保留”的定义

你的目标是：站控端只要“地图位置 + 基础状态”，不看实时姿态球。对应到 MAVLink 的最小信息集合就是：

- 链路存活/识别（否则 QGC/站控无法区分与保持连接）
- 全局位置（经纬度/高度/速度等最低限度）
- 基础健康/电量（电池、系统状态、解锁/落地等）

PX4 的 MAVLink stream 名称来自 MAVLink 模块的流类（`MavlinkStream`）注册体系；可用的流（stream 名称）在 `src/modules/mavlink/mavlink_messages.cpp` 中集中包含与注册（例如 HEARTBEAT、SYS_STATUS、GLOBAL_POSITION_INT 等都有对应 stream 类）。citeturn13view0  

另外，mavlink-router 项目也强调 **heartbeat 是“最关键”消息**，并给出 1Hz heartbeat 的工程建议（周期不超过 1000ms 即 1Hz）。citeturn53search1  

### 必须保留的核心消息与建议最低频率

下表给出“30 机集群（仅位置监控/基础状态）”建议白名单。频率是“最低安全/可用下限”的工程建议：你可以从下限开始压测，再按需要上调（比如地图轨迹平滑度不够就把位置 1Hz 提到 2Hz）。

| MAVLink 消息 (stream 名称) | 必要性 | 建议最低频率（每机） | 用途与说明 |
|---|---|---:|---|
| `HEARTBEAT` | 必须 | 1 Hz | 链路存活、系统类型、模式/解锁状态的基础载体。heartbeat 被认为是最关键消息之一，1Hz 常见且满足低带宽链路。citeturn53search1turn26view1 |
| `GLOBAL_POSITION_INT` | 必须 | 1 Hz（建议 1–2 Hz） | 地图位置监控核心。默认脚本里该消息曾被设到 50Hz（对集群极不友好），必须降频。citeturn26view1turn13view0 |
| `SYS_STATUS` | 强烈建议保留 | 0.5–1 Hz | 基础系统状态/传感器健康/电源等概要（不同地面站显示可能不同）。对应 stream 类在 mavlink_messages.cpp 注册体系内。citeturn13view0 |
| `BATTERY_STATUS` | 强烈建议保留 | 0.5–1 Hz | 电池电量/电压/电流等（比 SYS_STATUS 更专用）。citeturn13view0 |
| `EXTENDED_SYS_STATE` | 建议保留 | 0.2–1 Hz | landed state、VTOL 状态等扩展状态（用于“是否在地面/是否空中”等判断）。citeturn13view0 |
| `GPS_RAW_INT` | 可选（看你站控需要） | 0.5–1 Hz | 如果站控侧需要 GPS fix/sat/HDOP 等“定位质量”指标则保留。citeturn13view0 |
| `HOME_POSITION` | 可选 | 0.05–0.2 Hz 或事件触发 | 用于显示返航点/Home。若你不需要，可关。citeturn13view0 |
| `STATUSTEXT` | 建议保留（事件型） | 事件触发 | 虽不是固定流，但对故障/告警很有价值；不要主动高频推送。citeturn13view0 |

关键点：**位置**只需要 `GLOBAL_POSITION_INT` 低频即可；不用姿态球就不需要 `ATTITUDE*`。默认脚本把 `GLOBAL_POSITION_INT`/`ATTITUDE*`/`LOCAL_POSITION_NED` 等推到 50Hz，是你“5 架就拥堵”的主要原因之一。citeturn26view1  

### 强烈建议关闭或大幅降频的高频耗流消息

这些消息在 `px4-rc.mavlink` 默认设成 50Hz/10Hz（典型“单机调试”配置），在 30 机集群中应作为“第一批关闭对象”。citeturn26view1turn52view1  

| MAVLink 消息 (stream 名称) | 默认脚本频率（示例） | 集群监控建议 |
|---|---:|---|
| `ATTITUDE` / `ATTITUDE_QUATERNION` / `ATTITUDE_TARGET` | 50 Hz | 关闭（`-r 0`）。你明确“不看姿态球”，可直接砍掉。citeturn26view1 |
| `LOCAL_POSITION_NED` | 50 Hz | 关闭或降到 1Hz 以下（一般监控用不上 NED）。citeturn26view1 |
| `POSITION_TARGET_LOCAL_NED` | 50 Hz | 关闭（目标点/设定值用于调试/控制可视化，纯监控不需要）。citeturn26view1 |
| `RC_CHANNELS` | 50 Hz | 关闭或极低频（除非你要监视遥控输入）。citeturn26view1 |
| `SERVO_OUTPUT_RAW_0` | 50 Hz | 关闭（执行器输出监控一般只在调试需要）。citeturn26view1turn13view0 |
| `OPTICAL_FLOW_RAD` | 10 Hz | 关闭（非必要传感器调试流）。citeturn26view1 |

如果你在固件里还启用了其它常见高频流（例如 `HIGHRES_IMU`），也应按照同样原则关闭。PX4 文档示例展示了如何通过 `mavlink stream` 启用 `HIGHRES_IMU` 50Hz；在你这个场景应反其道而行之——不用就不要启用。citeturn52view0turn52view1  

## 固件级配置方法

### 通过启动脚本全局降频与白名单化

你的“极致压缩”最推荐做法是：把遥测链路用 `custom` 或 `minimal` 模式启动，然后用 `mavlink stream` 只保留白名单流，其他全部设为 0。

PX4 v1.16 文档给出 `mavlink start` / `mavlink stream` 的参数语义：

- `mavlink start -m <mode>`：选择模式（决定默认 streams 与默认频率），可选值包含 `custom|onboard|osd|config|minimal|...`，其中 `minimal` 为低带宽遥测常用模式，`custom` 适合“默认不推流、全靠你手工配置”。citeturn52view0turn52view1  
- `mavlink start -r <B/s>`：设置最大发送速率（B/s），超过则 sender 会降低 `rate mult`。citeturn52view0turn52view2  
- `mavlink stream -r <Hz>`：对单个 stream 设定频率，`0`=关闭，`-1`=默认。citeturn52view1turn52view2  

以 `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` 为例，你可以把默认的“50Hz 大套餐”删掉，仅留下白名单（伪代码示例，核心是思路）：

```sh
# 1) 启动 mavlink：强制低带宽
#    -m custom: 默认不启用 streams
#    -r 3000: 每机最大 3000 B/s（示例值，需压测）
mavlink start -u $udp_gcs_port_local -m custom -r 3000

# 2) 白名单：只保留位置 + 基础状态
mavlink stream -u $udp_gcs_port_local -s HEARTBEAT         -r 1
mavlink stream -u $udp_gcs_port_local -s GLOBAL_POSITION_INT -r 1
mavlink stream -u $udp_gcs_port_local -s SYS_STATUS        -r 1
mavlink stream -u $udp_gcs_port_local -s BATTERY_STATUS    -r 1
mavlink stream -u $udp_gcs_port_local -s EXTENDED_SYS_STATE -r 1
# 可选：
mavlink stream -u $udp_gcs_port_local -s GPS_RAW_INT        -r 1
mavlink stream -u $udp_gcs_port_local -s HOME_POSITION      -r 0  # 不要就关
```

对照 v1.16.0 的默认脚本，你会看到它原本把 `GLOBAL_POSITION_INT`、`ATTITUDE*`、`LOCAL_POSITION_NED` 等都推到 50Hz；你的改动目标就是把这些高频流删掉/关闭。citeturn26view1turn52view1  

补充两点“固件级减负”细节：

- 默认脚本在 `mavlink start` 里使用了 `-x`（Enable FTP）和 `-f`（Enable forwarding）。`-x`/`-f` 都会引入额外行为与潜在流量（例如文件/转发），在“低带宽集群监控链路”上通常应关闭，除非你确实需要。citeturn26view1turn52view2  
- 如果你使用广播（例如跨网段/WSL2 到 Windows QGC），`mavlink start -p` 可以打开 broadcast。PX4 讨论区里也有针对 SITL 广播链路（`-p`）的实践讨论。citeturn52view0turn17search7  

### 通过 `.px4board` 与 Kconfig 控制 MAVLink dialect（编译期裁剪）

你提到希望通过 `.px4board` 进行固件裁剪（例如 `CONFIG_MAVLINK_DIALECT`）。在 PX4 v1.16.0 中，MAVLink 模块的 Kconfig 明确提供了 `MAVLINK_DIALECT` 选项，默认值为 `"common"`。citeturn48view1  

这意味着你有两种编译期策略：

- **保持 `common`**：对 QGC、常用 SDK 兼容性最强（工程上最稳）。citeturn48view1turn55view3  
- **改成更“瘦”的 dialect（例如 minimal 或自定义）**：可减少生成的 MAVLink 头文件集合与固件中可用消息范围，从而减少代码体积与误用空间。但必须确保你保留的消息仍能满足站控/协议需求（尤其是你仍想用 QGC 作为站控时）。citeturn48view1  

`.px4board` 文件位置通常为 `boards/<vendor>/<board>/default.px4board`。以主分支的 FMUv6x 为例（文件格式与写法可参考），其中会启用 MAVLink 模块（`CONFIG_MODULES_MAVLINK=y`）。citeturn32view1  

实操上，你可以在目标板的 `default.px4board` 中加入/覆盖类似配置（示例）：

```ini
CONFIG_MODULES_MAVLINK=y
CONFIG_MAVLINK_DIALECT="common"   # 或 "minimal" / 你的自定义 dialect
```

注意：**dialect 的变更不会自动帮你“把发送频率降下来”**；它解决的是“可用消息集合/代码规模”问题。真正决定链路带宽的是启动脚本/参数对 stream 的开启与频率配置（上一节）。citeturn52view1turn48view1  

## 多机 SITL 仿真压测方案

### 关键网络事实与端口规划

PX4 v1.16 文档给出 SITL 的默认 MAVLink UDP 端口约定：

- GCS（QGC）通常监听 PX4 的 **远端** UDP 端口 `14550`。citeturn55view3  
- Offboard API 通常监听 PX4 的 **远端** UDP 端口 `14540`。citeturn55view3  
- 多机仿真时（开发者 API 侧）远端端口会在 `14540–14549` 顺序分配，更多实例会复用 `14549`。citeturn55view1turn55view3  
- `px4-rc.mavlink` 具体实现中，GCS 本地端口会按实例号分配为 `18570 + px4_instance`，并统一发往远端 `14550`。citeturn26view1turn55view3  

这对你的“30 机压力测试”非常有利：你可以让 **30 个 PX4 实例全部发往 QGC 的 14550**，而每实例使用不同本地端口，QGC 依靠 sysid（以及源端口/连接）来区分多载具。

### WSL2 (Ubuntu 20.04) + QGC 的推荐拓扑

在 WSL2 场景里，常见做法是：

- PX4 SITL（Gazebo）跑在 WSL2 内（Linux）
- QGC 5.0.8 跑在 Windows Host（桌面）
- 使用 `mavlink-routerd` 把 WSL2 内部回环 `127.0.0.1:14550` 的数据转发到 Windows Host 的 IP:14550

PX4 v1.16 仿真文档直接给出了 mavlink-router 的用法示例：把 SITL（发往 localhost 14550）的 MAVLink 流量转发到另一台机器上运行的 QGC（例如 `10.73.41.30:14550`）。citeturn55view2turn55view3  

示例（把 `10.73.41.30` 替换成你的 Windows Host 在 WSL2 可达的 IP）：

```sh
mavlink-routerd -e <WIN_HOST_IP>:14550 127.0.0.1:14550
```

该命令的语义也与 mavlink-router 官方 README 一致：`-e` 可添加多个 UDP endpoint，最后一个无 key 的参数可作为 UDP server 端点（等待连接/收包）。citeturn55view2turn53search1  

### 启动多机并压测带宽的操作步骤

在“新版 Gazebo（gz）”下，PX4 v1.16 仿真文档给出启动示例：

- 启动 Gazebo + x500：`make px4_sitl gz_x500`citeturn55view3  

你要做的是把单机扩展到 30 机，并观察“裁剪前后”的带宽差异。即使你最终的多机启动方式来自脚本（例如你自己写一个循环启动多个实例），压测的关键观测点仍是：

- 每个实例的 mavlink `-r <B/s>` 限制是否足够低（`mavlink status` 的 `rate mult` 是否接近 1、是否仍出现限流）citeturn52view0turn52view2  
- QGC 侧是否能同时稳定显示多载具，并且 `GLOBAL_POSITION_INT` 等关键消息频率是否符合你设定

由于 PX4 的脚本支持通过实例号变化端口（`px4_instance` 会影响端口计算），多机的核心是保证每实例 sysid 唯一、以及端口不冲突。脚本层面端口逻辑可直接参考 `px4-rc.mavlink` 的 `18570+px4_instance` 与 offboard 口的分配写法。citeturn26view1turn22view2  

### 在 QGC 中验证“压缩效果”的方法

QGroundControl 官方文档说明：**MAVLink Inspector** 会列出当前载具接收到的所有消息、source component id 以及 **update frequency（更新频率）**，并可下钻查看每个消息的字段值。打开路径是 Analyze View → MAVLink Inspector。citeturn53search0  

因此你可以用如下验证闭环：

- 压缩前：在 MAVLink Inspector 里观察 `ATTITUDE`、`GLOBAL_POSITION_INT` 等是否存在 50Hz 级别更新  
- 压缩后：确认 `ATTITUDE*` 等不再出现或频率为 0，`GLOBAL_POSITION_INT` 稳定在 1–2Hz，`HEARTBEAT` 约 1Hz，系统/电池类低频稳定  

同时，在 PX4 侧用 `mavlink status` 看 sender 是否出现 `rate mult < 1` 的被动限流（说明你仍然推流过多或 `-r` 设得过低导致被迫降频）。citeturn52view0turn52view2  

> 说明：你提到要在 QGC 里看“全局 Bitrate”。v1.16 PX4 官方与 QGC 官方用户手册中，明确可引用且稳定的验证方式是“消息频率/消息列表（MAVLink Inspector）+ PX4 侧 rate mult/状态（mavlink status）”。citeturn53search0turn52view0  
> 若你需要“单链接总比特率”的精确 UI 位置，建议以 QGC 5.0.8 的实际界面为准（不同版本菜单/Widget 名称可能变化），但不影响你用 Inspector 做压缩前后对比的工程结论。

## 额外工程建议（面向 10–30 机极限压缩）

第一，**强烈建议把“监控链路”和“控制链路”做逻辑隔离**：哪怕物理上仍是一根数传，也要在固件侧把“监控白名单流”与“控制/调参/任务”时段区分开（例如平时只开白名单，发任务前短时打开少量额外流，完成后再关闭），避免 30 机同时做参数/任务相关交互导致拥堵。

第二，如果你决定继续沿用 `px4-rc.mavlink` 的默认多机端口体系，要特别注意 **offboard 端口 10 架以上复用 14549** 的事实。你的场景若完全不依赖 offboard API，可直接关闭 offboard 实例/端口，使得最复杂的端口冲突问题自然消失。citeturn55view1turn26view1  

第三，mavlink-router 本身非常适合作为“站控侧汇聚器”：一方面可把 WSL2/容器/多网卡环境的 UDP 连接稳定地转发到 QGC，另一方面它支持多 endpoint、并且官方文档对配置目录、命令行 `-e` 添加多个 UDP endpoint 等都给出了明确用法。citeturn55view2turn53search1