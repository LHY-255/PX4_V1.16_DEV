# PX4 v1.16 Work Queue 模块化开发与代码模板深度研究报告

## 执行摘要

PX4 v1.16 推荐将多数自定义控制/功能模块实现为 **Work Queue（WQ）任务**（Work Queue task / ScheduledWorkItem），而不是创建“后台常驻线程（独立 task）”。其核心原因在于：WQ 通过 **集中式线程池（按队列划分）+ 可配置优先级/栈大小** 的方式，将多个模块复用同一工作线程与栈，从而显著降低 RAM/上下文切换开销，并且将调度行为与 PX4 的实时控制链路更自然地对齐（例如与姿态/位置控制等队列同优先级域）。PX4 官方架构文档明确指出，WQ 任务的优势在于 **更少 RAM、潜在更少任务切换**，但也强调 **禁止 sleep/poll/阻塞 IO**，重计算应考虑独立任务或独立队列。 citeturn13search13turn13search6turn16view0

在源码层面，PX4 v1.16 的 WQ 由 `WorkQueueManager` 统一管理：队列配置包含 `name/stacksize/relative_priority`，并以 **`SCHED_FIFO` + `sched_get_priority_max(SCHED_FIFO) + relative_priority`** 计算实际优先级；WQ 管理器任务名为 `wq:manager`，并提供 `WorkQueueManagerStart/Stop/Status`；命令行工具 `work_queue status` 直接调用 `WorkQueueManagerStatus()` 输出各工作队列负载信息。 citeturn18view0turn28view0turn35view0turn17view2

本报告交付以下可复用成果：  
1) 给出 WQ 相对后台线程的关键设计属性对比表，并结合 v1.16 源码解释优劣；2) 提供一个**最小可落地的 WQ C++ 模块模板**（`.hpp/.cpp/CMakeLists.txt/Kconfig`，并额外给出参数定义 `.c` 文件以满足 QGC 参数元数据生成），代码注释标注关键 API/路径（v1.16）；3) 以“编队控制”为例设计 uORB 通信流、频率与时延策略、消息融合/滤波与 Offboard 平滑介入/退出逻辑（含 mermaid 时序/流程图）；4) 给出启动脚本注册/启动方式（固件内置脚本与 SD 卡 `extras.txt` 两种方案），及在 QGC 5.0.8 下通过 MAVLink/Offboard 验证的测试清单；5) 给出 SITL/HIL/飞行前检查与性能监测指标（CPU/栈/延迟/丢期）。 citeturn31search1turn33search7turn13search1turn13search8turn28view0

---

## 为何 v1.16 自定义控制模块优先选择 Work Queue

### 任务模型与调度语义

PX4 官方文档将应用分为两类：  
一类是**独立 task（有自己的栈与进程优先级）**；另一类是**Work Queue task（运行在某个 Work Queue 线程上，与同队列模块共享栈与线程优先级）**。官方“模块模板”与“架构概览”均强调：多数场景建议用 Work Queue task 来最小化资源占用。 citeturn13search6turn13search13turn26search2

在 v1.16 源码中，Work Queue 的关键机制是：  
- `px4::WorkItem` 提供 `ScheduleNow()` 将自身加入队列执行；并可“切换到不同 WorkQueue”。 citeturn17view0turn17view1  
- `px4::ScheduledWorkItem` 在 `WorkItem` 基础上增加周期/延时调度接口（如 `ScheduleDelayed()`、`ScheduleOnInterval()` 等）。 citeturn17view3turn13search13  
- WQ 管理器 `WorkQueueManager` 维护队列配置：`wq_config_t{name, stacksize, relative_priority}`，并内置多组队列（例如：`rate_ctrl`、`nav_and_controllers`、`hp_default`、`lp_default` 等）。其中 `rate_ctrl` 被注释为“PX4 inner loop highest priority”。 citeturn35view0  
- WQ 线程优先级在创建时使用 FIFO 调度策略，并以 `sched_get_priority_max(SCHED_FIFO) + relative_priority` 计算优先级；因此 **relative_priority 越接近 0 越“高”**（例如 `rate_ctrl`=0 高于 `nav_and_controllers`=-13，高于 `lp_default`=-50）。 citeturn17view2turn35view0  

这意味着：当你开发“控制类模块”（尤其是要与姿态/位置控制闭环对齐）时，选择合适的 WQ（如 `nav_and_controllers` 或更高优先级域）可以在系统调度层面减少不可控的优先级漂移，而不必为每个模块单独创建高优先级任务并支付额外栈/RAM 成本。 citeturn35view0turn13search13

### 与 uORB 回调触发的天然耦合

WQ 模块常见触发方式有两种：定时调度或 uORB 更新回调。官方架构概览指出：WQ 任务可通过“指定未来的固定时间”或“uORB topic update callback”来调度。 citeturn13search13

v1.16 源码中，`uORB::SubscriptionCallbackWorkItem` 的 `call()` 实现会在订阅更新满足条件时，直接调用 `_work_item->ScheduleNow()` 触发该模块的 Run/周期函数执行；并支持 `set_required_updates()`（要求队列里累积到一定更新数才触发）。这为控制模块提供了更“事件驱动”的调度方式，减少无效轮询。 citeturn20view0turn17view0

### WQ 与后台线程的关键维度对比表

| 维度 | Work Queue 任务（ScheduledWorkItem/WorkItem） | 后台常驻线程（独立 task） |
|---|---|---|
| 实时性与调度可控性 | 依赖所在 WorkQueue 的线程优先级；v1.16 使用 FIFO，并按 `max_FIFO_priority + relative_priority` 计算线程优先级，适合与控制链路对齐（如 `rate_ctrl`、`nav_and_controllers`）。 citeturn17view2turn35view0 | 每个任务可单独设定进程优先级；更灵活，但容易造成“优先级碎片化”，且任务数过多会增加调度开销。官方文档暗示 WQ 可减少切换。 citeturn13search13 |
| 资源占用（RAM/栈/上下文切换） | 多模块共享队列线程与栈，官方明确“更少 RAM、潜在更少 task switches”。 citeturn13search13turn13search6 | 每任务独立栈与上下文；更高 RAM 占用与更多任务切换风险。 citeturn13search13 |
| 阻塞能力（sleep/poll/阻塞 IO） | **禁止** sleep/poll/阻塞 IO；长时间运行会拖慢同队列其他模块，官方建议重计算用独立任务或至少独立队列。 citeturn13search13turn16view0 | 允许阻塞等待（如 `poll()`、文件 IO、网络 IO）；适合必须阻塞的外设/通信场景。 citeturn13search13 |
| 可重入性与并发风险 | 同一 WorkItem 的 `Run()` 不会并发重入；但同队列不同模块共享线程，**任何一个模块超时会“连坐”**。需严格控制单次运行预算。 citeturn17view0turn17view1turn13search13 | 서로 독립；某任务超时不直接阻塞其他任务（但会抢占 CPU）。更好做隔离但更耗资源。 citeturn13search13 |
| 与 px4_work_queue / px4_task 交互 | WQ 管理器由 `px4_task_spawn_cmd("wq:manager", ...)` 启动；每个队列线程在 POSIX/Flat NuttX 用 pthread 创建，或在特定构建用 `px4_task_spawn_cmd`。citeturn18view0turn17view2 | 典型模块自己在 `task_spawn()` 中调用 `px4_task_spawn_cmd()` 创建独立任务（ModuleBase 文档说明）。 citeturn16view0 |
| 与 nsh/rcS 启动流程关系 | 模块仍通过启动脚本 `commander start` 等方式启动；可在系统启动后用 `extras.txt` 再启动自定义模块（无需固件重编译）。 citeturn12view0turn31search1 | 同上；但线程数增加可能触发系统 task 上限问题（社区也常见“任务表耗尽”类问题）。citeturn18view0turn26search21 |
| 观测与诊断 | v1.16 提供 `work_queue status`（systemcmds/work_queue）输出队列负载；`uorb top` 可观测 topic 频率与丢失；`load_mon` 在低优先级队列周期计算 CPU/RAM 并发布 `cpuload`。 citeturn28view0turn13search1turn13search8 | 可用 `top/perf` 等通用工具，但线程多时诊断复杂度上升。 (背景推断，仍建议以 `work_queue status` 为主) citeturn28view0turn23view0 |

结论：若你的自定义“控制模块”满足 **非阻塞、周期性/事件驱动、需要与控制链路优先级域对齐、并希望尽量节省 RAM/上下文切换**，则在 v1.16 上优先选 WQ；只有当模块必须阻塞 IO、或计算量不可预测且可能长时间占用 CPU，才应考虑独立 task 或将其隔离到单独 WorkQueue。 citeturn13search13turn35view0

---

## v1.16 最小化 Work Queue C++ 模块模板

本节给出一个“可复制的最小骨架”，目标是：  
- 使用 v1.16 的 `ModuleBase` + `ModuleParams` + `ScheduledWorkItem` 组合；citeturn16view0turn17view3  
- 具备参数定义（QGC 可见的元数据与固件默认值）、uORB 订阅/发布、性能计时与日志规范；citeturn34search26turn23view0turn23view1turn34search9  
- CMake/Kconfig 符合 v1.16 `px4_add_module()` 结构；citeturn28view1turn10view1turn11view5  

### 目录结构建议

建议新建目录（示例名：`formation_ctrl`）：

```
PX4-Autopilot/
  src/modules/formation_ctrl/
    CMakeLists.txt
    Kconfig
    formation_ctrl.hpp
    formation_ctrl.cpp
    formation_ctrl_params.c    (建议添加：用于 PARAM_DEFINE_* 元数据)
```

> 备注：严格按用户要求只必须交付 `.hpp/.cpp/CMakeLists.txt/Kconfig`，但**若要让参数出现在 v1.16 参数元数据/QGC 参数列表**，通常需要 `PARAM_DEFINE_*` 定义与注释元数据；官方参数文档也以 `PARAM_DEFINE_*` 为核心入口。 citeturn22search2turn34search26

### CMakeLists.txt 示例

```cmake
# src/modules/formation_ctrl/CMakeLists.txt
# 关键模式参考：src/systemcmds/work_queue/CMakeLists.txt 使用 px4_add_module() citeturn28view1

px4_add_module(
    MODULE modules__formation_ctrl
    MAIN   formation_ctrl
    SRCS
        formation_ctrl.cpp
        formation_ctrl.hpp
        formation_ctrl_params.c
    DEPENDS
        px4_work_queue   # WorkQueue/ScheduledWorkItem 所在库（命名随构建系统）
)
```

> 说明：`work_queue` systemcmds 的 CMakeLists 显示 v1.16 使用 `px4_add_module(MODULE ... MAIN ... SRCS ...)` 模式。 citeturn28view1  

### Kconfig 示例

参考 v1.16 自带的 `src/examples/work_item/Kconfig` 与 `src/systemcmds/work_queue/Kconfig` 的写法：使用 `menuconfig` 打开/关闭模块；必要时在 Protected build 下放入 userspace。 citeturn10view1turn28view2

```Kconfig
# src/modules/formation_ctrl/Kconfig

menuconfig MODULES_FORMATION_CTRL
    bool "formation_ctrl"
    default n
    ---help---
        Enable formation_ctrl module (Work Queue based).

if MODULES_FORMATION_CTRL

config USER_FORMATION_CTRL
    bool "formation_ctrl running as userspace module"
    default y
    depends on BOARD_PROTECTED
    ---help---
        Put formation_ctrl in userspace memory (protected builds).

endif # MODULES_FORMATION_CTRL
```

### 参数定义文件 formation_ctrl_params.c

该文件给出 QGC 可见的参数元数据（注释块）与固件默认值。官方参数文档明确：`PARAM_DEFINE_*` 宏指定类型、名称（需与代码中一致）以及固件默认值；注释元数据用于地面站显示与编辑约束。 citeturn22search2turn34search26

```c
// src/modules/formation_ctrl/formation_ctrl_params.c

#include <px4_platform_common/param.h>

/**
 * Formation control enable
 *
 * @boolean
 * @group Formation
 */
PARAM_DEFINE_INT32(FC_FORM_EN, 0);

/**
 * Desired formation spacing (m)
 *
 * @unit m
 * @min 0.5
 * @max 50.0
 * @decimal 2
 * @group Formation
 */
PARAM_DEFINE_FLOAT(FC_FORM_D, 5.0f);

/**
 * Position error proportional gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Formation
 */
PARAM_DEFINE_FLOAT(FC_FORM_KP, 1.0f);
```

### 头文件 formation_ctrl.hpp

```cpp
// src/modules/formation_ctrl/formation_ctrl.hpp

#pragma once

#include <px4_platform_common/module.h>                 // ModuleBase<>（见 module.h 的 work-queue 模式说明）
#include <px4_platform_common/module_params.h>          // ModuleParams
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp> // ScheduledWorkItem（WorkItem 的定时调度）
#include <px4_platform_common/log.h>                    // PX4_INFO/WARN/ERR

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>               // SubscriptionCallbackWorkItem -> ScheduleNow()
#include <uORB/Publication.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <perf/perf_counter.h>                          // perf_alloc/begin/end/print（见 perf_counter.h）

using namespace time_literals;

class FormationCtrl final
    : public ModuleBase<FormationCtrl>
    , public ModuleParams
    , public px4::ScheduledWorkItem
{
public:
    FormationCtrl();
    ~FormationCtrl() override;

    /** ModuleBase 必需接口
     * 参考：platforms/common/include/px4_platform_common/module.h
     * - work queue 模式需在 task_spawn() 中设置 _task_id = task_id_is_work_queue
     * - 退出时需在 cycle/Run 中调用 exit_and_cleanup()
     */
    static int task_spawn(int argc, char *argv[]);
    static FormationCtrl *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int print_status() override;

private:
    void Run() override;               // ScheduledWorkItem 回调：禁止阻塞（架构文档强约束）
    bool init();                       // 注册 uORB 回调/定时调度

    void parameters_update(bool force = false);
    void publish_setpoints(const vehicle_local_position_s &lpos);

private:
    // 参数更新触发：常用 pattern（间隔订阅）
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // 关键状态订阅（可按需扩展：vehicle_control_mode/vehicle_odometry 等）
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

    // 若希望由 lpos 更新触发执行，可用 SubscriptionCallbackWorkItem
    // 源码：platforms/common/uORB/SubscriptionCallback.hpp -> SubscriptionCallbackWorkItem::call() 调度 WorkItem::ScheduleNow()
    uORB::SubscriptionCallbackWorkItem _lpos_trigger{this, ORB_ID(vehicle_local_position)};

    // 输出：Offboard setpoint 典型组合（offboard_control_mode + trajectory_setpoint）
    // OffboardControlMode 消息字段定义：msg_docs/OffboardControlMode
    uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
    uORB::Publication<trajectory_setpoint_s>   _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

    // 性能计时（见 src/lib/perf/perf_counter.h）
    perf_counter_t _cycle_perf{nullptr};
    perf_counter_t _cycle_interval_perf{nullptr};

    // 参数句柄（使用 Param* + updateParams() 机制）
    // 参数元数据来自 formation_ctrl_params.c 的 PARAM_DEFINE_*；访问接口来自 px4_platform_common/param.h
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::FC_FORM_EN>) _param_fc_form_en,
        (ParamFloat<px4::params::FC_FORM_D>) _param_fc_form_d,
        (ParamFloat<px4::params::FC_FORM_KP>) _param_fc_form_kp
    )

    // 状态
    bool _enabled{false};
    hrt_abstime _last_setpoint_pub{0};
};
```

关键点对齐说明：  
- `ModuleBase` 对 WQ 模块的 `task_spawn()`/退出约束在 `module.h` 注释中给出；尤其指出 WQ 模式要设置 `_task_id = task_id_is_work_queue`，并在循环/周期函数中调用 `exit_and_cleanup()`。 citeturn16view0  
- `SubscriptionCallbackWorkItem::call()` 在 topic 更新时调度 `WorkItem::ScheduleNow()`，这是实现“由 uORB 驱动 WQ”最关键的钩子。 citeturn20view0turn17view0  
- `offboard_control_mode` 与 `trajectory_setpoint` 是 PX4 Offboard 常用输入话题：Offboard 文档指出 Offboard 模式依赖持续“proof-of-life”与设定值流；并且 `OffboardControlMode` 字段明确区分 position/velocity/attitude/直接作动器等控制接口。 citeturn33search7turn33search8turn33search0  

### 源文件 formation_ctrl.cpp（最小运行逻辑）

```cpp
// src/modules/formation_ctrl/formation_ctrl.cpp

#include "formation_ctrl.hpp"

#include <px4_platform_common/tasks.h>  // px4_task_spawn_cmd 等（WQ manager 内部也使用）

FormationCtrl::FormationCtrl()
    // 选择合适 WorkQueue：
    // - v1.16 wq_configurations::nav_and_controllers：注释为“att/pos controllers, highest priority after sensors.”
    //   源码：platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp
    : ModuleParams(nullptr)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
    // perf 计数器：src/lib/perf/perf_counter.h
    _cycle_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
    _cycle_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME": interval");
}

FormationCtrl::~FormationCtrl()
{
    // 释放 perf
    perf_free(_cycle_perf);
    perf_free(_cycle_interval_perf);

    // 取消回调注册
    _lpos_trigger.unregisterCallback();
}

bool FormationCtrl::init()
{
    // 选择“事件驱动”：由 vehicle_local_position 更新触发 ScheduleNow()
    // SubscriptionCallbackWorkItem 注册过程：platforms/common/uORB/SubscriptionCallback.hpp
    if (!_lpos_trigger.registerCallback()) {
        PX4_WARN("lpos callback register failed, fallback to interval scheduling");
        // 定时调度作为降级：ScheduleOnInterval（ScheduledWorkItem.hpp）
        ScheduleOnInterval(20_ms); // 50Hz
    }

    // 也可以直接定时：ScheduleOnInterval(20_ms);
    return true;
}

void FormationCtrl::parameters_update(bool force)
{
    if (_parameter_update_sub.updated() || force) {
        parameter_update_s p{};
        _parameter_update_sub.copy(&p);

        // ModuleParams 更新：px4_platform_common/param.h DEFINE_PARAMETERS 宏生成的 updateParamsImpl()
        updateParams();

        _enabled = (_param_fc_form_en.get() > 0);
    }
}

void FormationCtrl::publish_setpoints(const vehicle_local_position_s &lpos)
{
    // OffboardControlMode：msg_docs/OffboardControlMode（position/velocity/attitude/... flags）
    offboard_control_mode_s ocm{};
    ocm.timestamp = hrt_absolute_time();
    ocm.position = true;
    ocm.velocity = false;
    ocm.acceleration = false;
    ocm.attitude = false;
    ocm.body_rate = false;
    ocm.thrust_and_torque = false;
    ocm.direct_actuator = false;
    _offboard_control_mode_pub.publish(ocm);

    // TrajectorySetpoint：msg_docs/TrajectorySetpoint（NED；NaN 表示该维度不控制）
    trajectory_setpoint_s sp{};
    sp.timestamp = ocm.timestamp;

    // 这里仅示例：保持当前位置（“无跳变”介入的最小策略）
    // 编队控制时 sp.x/sp.y/sp.z 应为 leader+offset、或相对误差反馈（见后文设计）
    sp.x = lpos.x;
    sp.y = lpos.y;
    sp.z = lpos.z;
    sp.yaw = NAN;

    _trajectory_setpoint_pub.publish(sp);

    _last_setpoint_pub = sp.timestamp;
}

void FormationCtrl::Run()
{
    perf_begin(_cycle_perf);
    perf_count(_cycle_interval_perf);

    // WQ 任务禁止阻塞：官方架构文档明确禁止 sleep/poll/阻塞 IO
    // 只能做“快进快出”的控制计算

    parameters_update(false);

    vehicle_local_position_s lpos{};
    if (_vehicle_local_position_sub.update(&lpos)) {

        if (_enabled) {
            publish_setpoints(lpos);
        }
    }

    // 退出条件处理（ModuleBase::should_exit()）：work queue 模式需要在 Run() 中触发清理
    if (should_exit()) {
        // ModuleBase::exit_and_cleanup() 说明：platforms/common/include/px4_platform_common/module.h
        exit_and_cleanup();
        return;
    }

    perf_end(_cycle_perf);
}

int FormationCtrl::print_status()
{
    PX4_INFO("enabled=%d", _enabled);
    perf_print_counter(_cycle_perf);
    perf_print_counter(_cycle_interval_perf);
    return 0;
}

// ModuleBase 接口实现（最小化）
int FormationCtrl::task_spawn(int argc, char *argv[])
{
    // work queue 模式：ModuleBase 注释要求 _task_id = task_id_is_work_queue（见 module.h）
    // 通常做法：instantiate -> init -> _object 已设置后返回
    _task_id = task_id_is_work_queue;

    // 解析参数（略）：可在 instantiate() 中做
    return 0;
}

FormationCtrl *FormationCtrl::instantiate(int argc, char *argv[])
{
    auto *instance = new FormationCtrl();
    if (instance && !instance->init()) {
        delete instance;
        instance = nullptr;
    }
    return instance;
}

int FormationCtrl::custom_command(int argc, char *argv[])
{
    return print_usage("unrecognized command");
}

int FormationCtrl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Formation control example module running on a Work Queue (ScheduledWorkItem).
Publishes OffboardControlMode + TrajectorySetpoint when enabled.

### Examples
Enable and start:
$ param set FC_FORM_EN 1
$ formation_ctrl start
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("formation_ctrl", "controller");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

// 模块入口：与其它模块一致
extern "C" __EXPORT int formation_ctrl_main(int argc, char *argv[])
{
    return FormationCtrl::main(argc, argv);
}
```

该模板遵循 v1.16 的关键规范与来源：  
- WQ 队列选择：`wq_configurations::nav_and_controllers` 等队列在 `WorkQueueManager.hpp` 中定义，并带栈大小与相对优先级注释。 citeturn35view0  
- WQ 调度触发：`SubscriptionCallbackWorkItem::call()` -> `WorkItem::ScheduleNow()`。 citeturn20view0turn17view0  
- WQ 任务限制：架构文档明确 WQ 任务不可 sleep/poll/阻塞 IO，重计算建议独立任务/独立队列。 citeturn13search13turn16view0  
- 性能计数器 API：`perf_alloc/begin/end/print_counter` 在 `perf_counter.h` 中定义。 citeturn23view0  
- offboard 输入消息：`OffboardControlMode`/`TrajectorySetpoint` 与其语义在 msg_docs 中给出。 citeturn33search8turn33search0  
- 日志规范：`PX4_INFO/WARN/ERR` 宏在 `log.h` 中定义。 citeturn23view1  

---

## 编队控制示例设计：uORB 通信流、频率与 Offboard 平滑介入

### 编队控制的“机内模块”定位假设

本报告将“编队控制模块”定位为：运行在 PX4 飞控内（WQ），在需要时向现有控制链路发布设定值（setpoints），使下游控制器（如 multicopter position controller）接管执行。这种方式的优点是：**不需要改 FlightTask/模式管理器**，只需在 Offboard 模式下提供持续 setpoint 流即可；风险是：你必须严格遵守 Offboard 的“proof-of-life”与 failsafe 约束。 citeturn33search7turn33search8turn33search0  

### 必须订阅的原生话题与建议扩展

下表列出“编队控制”常见订阅（v1.16 原生 uORB）。其中 `vehicle_local_position`/`vehicle_global_position`/`vehicle_attitude`/`vehicle_status` 是最常用的状态输入；它们的消息定义可在 v1.16 msg_docs 查到。 citeturn33search2turn35view0turn33search3turn34search0turn33search13

**本机状态输入（建议最小集合）**  
- `vehicle_local_position`：NED 融合本地位置（EKF2 启动时刻为原点）。citeturn33search2  
- `vehicle_global_position`：WGS84 融合全局位置（非原始 GPS）。citeturn34search0  
- `vehicle_attitude`：姿态四元数（机内使用，类似 MAVLink ATTITUDE_QUATERNION）。citeturn33search3  
- `vehicle_status`：由 commander 发布，编码系统状态（如 armed 状态等）。citeturn33search13  
- `vehicle_command`：用于接收模式切换/动作命令（例如由 MAVLink COMMAND_LONG/INT 映射）。citeturn15search27turn34search5  

**队友/编队状态（v1.16 原生能力评估）**  
- v1.16 **没有通用“teammate_* / swarm_*”原生 uORB 话题**作为标准编队数据总线（在官方 msg_docs 索引中也未见此类通用命名；至少在本次检索范围内无明确“teammate_*”条目）。citeturn14search0turn15search1  
- 可利用的“单目标跟随”相关话题：`FollowTarget`（用于 Follow-Me，携带目标经纬高与速度）。它更像“跟随单一目标”的输入，不等价于多机编队，但可作为“leader 广播”的最小复用点。citeturn15search2turn15search23  
- 推荐扩展建议（当需要多队友）：依据官方 uORB 文档，新增话题需要在 `msg/`（或 `msg/versioned/`）添加 `.msg` 并加入 `msg/CMakeLists.txt`，构建时自动生成 C/C++ 代码；也支持 out-of-tree 消息定义。citeturn34search9turn33search22  
  - 建议定义：`teammate_state.msg`（多实例或数组字段），包含 `sysid/compid`、`timestamp`、`pos/vel`（NED 或 LLH+NED）、`covariance`、`link_quality` 等。多机情况下优先考虑 **多实例（multi-instance uORB）** 或“数组 + 有效位”方式，结合 `uorb top` 监测丢包。citeturn13search1turn34search9  

### 应发布的控制话题与控制层级选择

编队控制模块的输出取决于你希望接入 PX4 的哪一层控制链路：

**推荐：发布位置/轨迹设定值（更高层，安全性更好）**  
- `offboard_control_mode`：声明使用 position/velocity/attitude 等通道。citeturn33search8turn33search7  
- `trajectory_setpoint`：NED 轨迹设定值输入 PID 位置控制器；文档指出 NaN 表示该维度不控制。citeturn33search0turn34search17  

**可选：发布角速率/推力设定值（更底层，风险更高）**  
- `vehicle_rates_setpoint`：包含 body thrust 等字段（多旋翼通常 thrust_body[2] 为负油门需求的约定说明在 msg_docs 中出现）。citeturn34search1  
- 更底层的 actuator 输出路径：v1.16 更推荐使用控制分配（control allocation）链路，将控制器输出的力矩/推力映射到 `ActuatorMotors/ActuatorServos` 等；控制分配概念文档说明 v1.14+ 用 control allocation 替代旧 mixing。citeturn34search23turn34search2  

> 对新项目而言，优先在轨迹/位置层实现编队；只有在你明确需要“编队队形保持的快速局部闭环”并且有成熟的安全约束时，才下沉到 rates/actuator 层。此建议源于 Offboard 风险与控制分配复杂度的综合权衡（Offboard 文档强调其危险性与 failsafe 依赖）。citeturn33search7turn34search23  

### 频率、QoS/时延与消息融合策略

**Offboard 生命信号与最低频率**  
PX4 Offboard 文档明确：必须以 **≥2Hz** 接收 MAVLink setpoint 或 ROS2 `OffboardControlMode` 作为外部控制器健康证明；并且需要至少持续发送 **>1 秒** 才能在 Offboard 下解锁或切换进入。若 setpoint 流低于 2Hz，PX4 将在 `COM_OF_LOSS_T` 超时后退出 Offboard，并按 `COM_OBL_RC_ACT`（是否有 RC）选择 failsafe 动作。 citeturn33search7turn24search11turn24search22  

**工程建议频率（编队场景）**  
- `offboard_control_mode` 与 `trajectory_setpoint`：建议 **20–50Hz**（典型与位置控制循环匹配，留足链路抖动）。MAVROS Offboard 示例也强调 setpoint 发布必须快于 2Hz，并提到 PX4 OFFBOARD 命令间 timeout 500ms（解释为何要更快以覆盖时延）。citeturn33search16turn24search15turn33search0  
- 队友状态（若经 MAVLink/无线 mesh 输入）：建议 10–20Hz 并在 본机端做时间戳对齐与预测（constant velocity / alpha-beta），以降低链路抖动造成的队形抖动。（此为工程推断：时延抖动对相对位置闭环影响显著；落地时可用 `uorb top`/日志验证）citeturn13search1turn15search19  

**消息融合/滤波策略（推荐“轻量、可证”）**  
- 相对位置误差：使用 `vehicle_local_position`（本机）与“队友位置（转换到同一坐标系）”计算。`vehicle_local_position` 明确是 NED 融合位置并以 EKF2 启动为原点，因此多机时必须解决“原点不一致”问题：  
  - 方案 A：全部用 `vehicle_global_position`（WGS84）做 leader 跟随与队形偏置（地理坐标下偏置再转 ENU/NED），再回写本机 `trajectory_setpoint` NED（需地理到 NED 的一致变换，工程复杂）。citeturn34search0turn33search0  
  - 方案 B：统一由 leader 广播“队形坐标系定义”（例如 leader 当前位置作为原点、航向定义 x 轴），各机将队友状态映射到 leader-frame。该方案需要自定义消息字段携带 frame 定义（推荐自定义 teammate topic）。citeturn34search9turn33search22  
- 平滑：对队友位置做一阶低通（EMA）或 alpha-beta 滤波；对 setpoint 做速度/加速度限制（保证 `trajectory_setpoint` 动力学可行）。官方对 `TrajectorySetpoint` 也强调需“kinematically consistent and feasible for smooth flight”。citeturn33search0  

### 平滑 Offboard 介入/切换逻辑（含优先级与 failsafe）

下面给出一个建议的“编队控制介入”逻辑：核心目标是避免 setpoint 跳变、避免 Offboard timeout、并能在队友丢失/操作者介入时快速退出。

```mermaid
flowchart TB
  A[模块启动/待机] --> B{FC_FORM_EN=1?}
  B -- 否 --> A
  B -- 是 --> C[进入Arming/Mode监视: vehicle_status / vehicle_command]
  C --> D{当前是否Offboard?}
  D -- 否 --> E[不发布setpoint或仅预热: 发送OffboardControlMode+TrajectorySetpoint(=当前位姿)  >1s]
  E --> D
  D -- 是 --> F{队友数据新鲜? <T_teammate_timeout}
  F -- 否 --> G[降级/退出: 保持位置setpoint 或触发退出Offboard(交给COM_OF_LOSS_T/COM_OBL_RC_ACT)]
  F -- 是 --> H[计算编队目标: leader+offset + 滤波/限速]
  H --> I[发布 OffboardControlMode(位置) + TrajectorySetpoint @20-50Hz]
  I --> J{操作者/RC覆盖? 或 vehicle_command Stop?}
  J -- 是 --> K[渐进释放: setpoint回到当前位置/或停止发布 -> 退出]
  J -- 否 --> F
```

与官方安全语义的对应关系：  
- Offboard 需要持续 >2Hz 的 setpoint/OCM 流；低于 2Hz 将触发 `COM_OF_LOSS_T` 超时退出并按 `COM_OBL_RC_ACT` failsafe。 citeturn33search7turn24search22  
- 建议从 Position 模式进入 Offboard 的理由：MAVROS Offboard 文档指出，掉出 Offboard 时会回到进入前模式，因此进入前模式若为 Position，可让飞行器“就地停悬”更安全。 citeturn33search16turn24search15  

---

## 启动脚本注册/启动示例与 QGC 5.0.8 Offboard 测试

### 启动脚本与 rcS/vehicle_setup 关系

v1.16 的启动由 `ROMFS/px4fmu_common/init.d/rcS` 作为入口脚本，负责调用后续脚本链；多旋翼等载具的控制器启动通常经 `rc.vehicle_setup` 分发到 `rc.mc_apps/rc.fw_apps/...`。 citeturn12view0turn29view2turn29view1

其中 `rc.mc_apps` 示例显示会启动 `flight_mode_manager`、`mc_pos_control` 等（注释中列出典型控制器链）。 citeturn29view1  

### 两种推荐启动方式

**方式一：固件内置脚本启动（需改 ROMFS 并重新编译）**  
在合适的脚本（例如 `rc.mc_apps`）末尾加入：

```sh
# In ROMFS/px4fmu_common/init.d/rc.mc_apps
# 编队控制模块：按参数决定是否启动（也可直接 start）
if param greater -s FC_FORM_EN 0
then
    formation_ctrl start
fi
```

该方式适合“你要把编队作为固件内置能力”的产品化路径；但需要维护 ROMFS 差异。  
（脚本文件位置与调用链路依据 rcS/vehicle_setup 结构）。 citeturn29view1turn29view2turn12view0  

**方式二（更推荐用于快速迭代）：SD 卡 `extras.txt` 动态扩展启动**  
官方系统启动文档提供 hook：在 SD 卡 `etc/extras.txt` 中写入启动命令，系统主启动完成后会执行，用于启动“额外应用（payload controller 等）”。并明确该文件在代码中路径为 `/fs/microsd/etc/extras.txt`。 citeturn31search1turn31search0  

`extras.txt` 示例：

```sh
# /fs/microsd/etc/extras.txt
formation_ctrl start
```

> 注意：官方警告在启动文件中调用未知命令可能导致启动失败。 citeturn31search1  

### QGC 5.0.8 下通过 MAVLink/Offboard 交互测试要点

本节聚焦“验证链路与模式切换”，而不是“QGC UI 是否直接生成编队 setpoint”（QGC 通常更像 GCS，复杂 setpoint 更常由 MAVSDK/MAVROS/自研 companion 发送）。Offboard 官方文档明确：setpoint 可由 MAVLink 或 MAVSDK/ROS2 提供。 citeturn33search7turn24search7  

**必须满足的 Offboard 基本条件**  
1) **持续发送 setpoint/OCM ≥2Hz**，且 **至少发送 1 秒以上** 才允许 Offboard 下解锁/切入。 citeturn33search7  
2) 若 setpoint 断流（<2Hz），PX4 在 `COM_OF_LOSS_T` 后退出 Offboard，并按 `COM_OBL_RC_ACT` 采取 failsafe。 citeturn24search22turn33search7  

**常用 MAVLink setpoint 消息（官方 ROS/MAVROS Offboard 文档给出）**  
- `SET_POSITION_TARGET_LOCAL_NED`、`SET_ATTITUDE_TARGET`：官方 Offboard Control（ROS）页明确指出通过 MAVLink 协议，特别使用这两类消息来做 offboard 控制。 citeturn24search14  

**QGC 侧建议验证动作（面向 v1.16）**  
- 使用 QGC 的 MAVLink Console/Analyze 工具观察模式与 setpoint 流（工程实践；文档侧可用“飞行前检查”页面强调 QGC 能给出无法解锁的精确原因）。citeturn24search29  
- 在参数侧确认 Offboard/安全相关参数（至少确认 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 存在并按需求配置）。Offboard 文档直接点名这两个参数。 citeturn24search22turn33search7  
- 如采用 MAVROS/MAVSDK 产生 setpoint：MAVROS Offboard 示例强调 setpoint 发布需快于 2Hz 且给出 20Hz 的典型发布循环；并提示 500ms 超时机制与从 Position 切入 Offboard 的安全建议。 citeturn33search16turn33search18  

---

## 测试验证步骤与性能监测指标

### 分阶段测试路线

**SITL（软件在环）优先**  
v1.16 仿真文档强调：仿真是测试 PX4 代码改动的快速、安全方式，可像真实飞行器一样用 QGC 或 offboard API 与之交互。 citeturn34search24turn31search12  
- 建议：先在 Gazebo SITL 跑通启动、参数、话题流与 Offboard 切换；随后再做多机仿真/编队扩展（多机仿真页会涉及实例与启动文件映射）。 citeturn31search24turn31search12  

**HIL（硬件在环）**  
若引入真实飞控硬件，HIL 可在“真实固件 + 仿真物理/传感器”下验证时序与资源占用。jMAVSim 文档说明仿真循环会发送 `HIL_SENSOR`，PX4 运行估计/控制并回发 `HIL_ACTUATOR_CONTROLS`。 citeturn34search29  

**飞行前检查清单（强制建议）**  
官方飞行前检查文档指出：QGC 能精确显示不能解锁的原因，是判断 readiness 的关键工具。编队控制属于高风险功能，务必在飞行前通过 QGC 的检查与提示确认状态。 citeturn24search29turn33search7  

### 性能与时序监测指标

**Work Queue 负载与丢期（missed deadlines proxy）**  
- 使用 `work_queue status`：v1.16 `systemcmds/work_queue` 明确提供 `start/stop/status`，其中 `status` 调用 `WorkQueueManagerStatus()` 输出 Work Queue 线程的 RATE/INTERVAL 等信息，可用于观察队列是否“跑不动”。 citeturn28view0turn18view0  
- 解释：WQ 共享线程，若你模块一次 Run 太久，会拉低同队列 RATE、增大 INTERVAL（这在工程上可视为“deadline miss”的外显信号）。该推断与官方“WQ 不应长时间运行”的约束一致。 citeturn13search13turn16view0  

**uORB 话题频率与丢失**  
- `uorb top`：官方 uORB 文档说明该命令可实时显示每个 topic 发布频率（Hz）、丢失消息数等，非常适合验证编队 setpoint 是否满足 >2Hz 且稳定。 citeturn13search1turn33search7  

**CPU/RAM/栈**  
- `load_mon`：模块参考指出其在低优先级 work queue 周期运行，计算 CPU load/RAM 并发布 `cpuload`；在 NuttX 还会检查每个进程栈余量，低于阈值会告警并写入日志。 citeturn13search8  
- perf 计数器：在模块内使用 `perf_alloc/begin/end/print_counter` 统计周期耗时与间隔，有助于量化“控制计算预算”。citeturn23view0turn29view1  

**日志与可追溯性**  
- 使用 `PX4_INFO/WARN/ERR` 记录状态切换、队友丢失、Offboard 退出原因；日志宏定义在 `log.h`。citeturn23view1turn33search7  
- 建议将编队关键输入（本机位姿、队友位姿、输出 setpoints、模式状态）加入日志主题集合，便于 ULog 离线复盘（ULog 格式文档说明其自描述与 uORB topic 日志能力）。 citeturn15search19turn13search1  

---

## 附录：关键源码与文档路径速查

### Work Queue 关键源码路径（v1.16.0 tag）

- WQ 配置（队列名/栈/相对优先级）：`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp`（`px4::wq_configurations::*`，如 `rate_ctrl/nav_and_controllers/hp_default/lp_default`）。 citeturn35view0  
- WQ 管理器启动/优先级计算/状态输出：`platforms/common/px4_work_queue/WorkQueueManager.cpp`（`WorkQueueManagerStart()` 启动 `wq:manager`；按 FIFO 优先级计算；`WorkQueueManagerStatus()` 输出线程状态）。 citeturn18view0turn17view2  
- WorkItem/WorkQueue 基类：`platforms/common/include/px4_platform_common/px4_work_queue/WorkItem.hpp`、`.../WorkQueue.hpp`。 citeturn17view0turn17view1  
- uORB 回调驱动 WQ：`platforms/common/uORB/SubscriptionCallback.hpp`（`SubscriptionCallbackWorkItem::call()` -> `WorkItem::ScheduleNow()`）。 citeturn20view0  
- 模块基类对 WQ 模式约束：`platforms/common/include/px4_platform_common/module.h`（说明 work queue 模式 spawn/exit 约定）。 citeturn16view0  
- `work_queue` CLI：`src/systemcmds/work_queue/work_queue_main.cpp`（`status` 调用 `WorkQueueManagerStatus()`）。 citeturn28view0  

### 编队/Offboard 关键 uORB 消息（v1.16 msg_docs）

- `VehicleLocalPosition`：融合本地 NED 位置。 citeturn33search2  
- `VehicleGlobalPosition`：融合 WGS84 全局位置。 citeturn34search0  
- `VehicleAttitude`：姿态四元数。 citeturn33search3  
- `VehicleStatus`：系统状态（commander 发布）。 citeturn33search13  
- `VehicleCommand`：动作/命令消息。 citeturn15search27turn34search5  
- `OffboardControlMode`：Offboard 控制通道选择/声明。 citeturn33search8  
- `TrajectorySetpoint`：NED 轨迹设定值（NaN 表示不控制该维度；需动力学可行）。 citeturn33search0turn34search17  
- `VehicleRatesSetpoint`：角速率/推力设定值（如需下沉控制层）。 citeturn34search1  
- `ActuatorMotors`：电机控制输出消息（控制分配链路下游）。 citeturn34search2turn34search23  
- （可复用的单目标跟随输入）`FollowTarget`：用于 Follow-Me 的目标位置/速度广播。 citeturn15search2turn15search23  

### 启动与调试文档（v1.16）

- 系统启动与 `extras.txt`：`/fs/microsd/etc/extras.txt` 可在主系统 boot 后启动自定义应用。 citeturn31search1turn31search0  
- Offboard 模式约束（>2Hz、>1s、`COM_OF_LOSS_T`、`COM_OBL_RC_ACT`）：Offboard 模式文档。 citeturn33search7turn24search22  
- uORB 调试（`uorb top`）：uORB 文档。 citeturn13search1turn34search9  
- `load_mon` 系统监测：模块参考 System。 citeturn13search8