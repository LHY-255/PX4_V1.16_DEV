# PX4 version：1.16
# 飞控：TS IFC1

***

# 任务：

1.针对PX4，1.16版本，QGC5.0.8，进行固件剪裁和bug修复工作，主要理解软件整体架构（操作系统、软件整体逻辑、MAVlink通信交互），飞控算法部分内容（包括外环、内环、ESC控制）、感知导航部分（EKF），整理出一个整体的代码解析readme，对固件对无关部分可以进行剪裁（其他模式的代码，不用的传感器的代码等，后续整理个详细需求）；

2.搭建一个算法的HIL仿真系统，可实现算法的在线仿真，对剪裁的固件进行测试仿真（开学前有一版本）；

3.搭建一个内环飞控测试台，可实现sim-to-real的算法验证；

***

# 一.固件裁剪

```
固件裁剪主要在/PX4-Autopilot/boards/px4/fmu-v6x/default.px4board文件中进行。通过配置模块的启动与否来控制其代码是否编译，以达到固件裁剪的目的。

/PX4-Autopilot/boards/px4/fmu-v6x/default.px4board

展示修改的部分：
CONFIG_COMMON_DIFFERENTIAL_PRESSURE=y   #压差传感器开启
CONFIG_DRIVERS_MS4525=y                  #空速计开启
CONFIG_DRIVERS_GNSS_SEPTENTRIO=n        #关闭这个GNSS
CONFIG_DRIVERS_GPS=y                   #适配NEO-M9N-00B GNSS
CONFIG_DRIVERS_IMU_BOSCH_BMI088=y        #适配BMI088
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20602=n   #关闭
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20649=n   #关闭
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=n   #关闭
CONFIG_DRIVERS_IMU_INVENSENSE_ICM42670P=n  #关闭
CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y  #适配ICM42688-P
CONFIG_DRIVERS_IMU_INVENSENSE_ICM45686=n   #关闭
CONFIG_DRIVERS_IMU_INVENSENSE_IIM42652=n    #关闭
CONFIG_MODULES_FW_ATT_CONTROL=y       #开启固定翼模块
CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL=y      #开启固定翼模块
CONFIG_MODULES_FW_POS_CONTROL=y     #开启固定翼模块
CONFIG_MODULES_FW_RATE_CONTROL=y    #开启固定翼模块
CONFIG_MODULES_MC_ATT_CONTROL=y    #开启多旋翼模块
CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL=y    #开启多旋翼模块
CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR=y   #开启多旋翼模块
CONFIG_MODULES_MC_POS_CONTROL=y    #开启多旋翼模块
CONFIG_MODULES_MC_RATE_CONTROL=y    #开启多旋翼模块
CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF=y     #开启VTOL(复合翼)的相关模块
CONFIG_MODULES_VTOL_ATT_CONTROL=y       #开启VTOL(复合翼)的相关模块

无人车、无人潜艇等模块是默认关闭的，可以不用设置。如果要设置，可以在
/PX4-Autopilot/boards/px4/fmu-v6x/default.px4board文件中显式开启，也可以在PX4/PX4-Autopilot/src/modules中对应模块的Kconfig文件中开启或者关闭。
```

**注意，在PX4代码中不要使用中文注释，会报错！**

***

# 二.rcS脚本详细解析(对照rcS文件阅读此章节)
---
- 飞控硬件上电并且完成`Bootloader`引导后，由实时操作系统`NuttX`调用第一个启动脚本：`PX4-Autopilot/ROMFS/px4fmu_common/init.d/rcS`，该脚本主要完成如下任务：
---
- 脚本环境与调试控制
```
#!/bin/sh  #指定脚本解释器为系统的shell程序
set +e  #设置“忽略错误继续执行”模式。这意味着即使某行命令运行失败，脚本也不会立即退出，确保系统能尽可能多的初始化硬件
#set -x  #此行被注释掉，若启用，系统会打印出脚本执行的每一条指令，通常用于底层调试
```
- 基础路径变量设置
```
set R /  #定义根目录变量${R}为 /
set FCONFIG /fs/microsd/etc/config.txt  #定义用户自定义配置文件的路径，位于 SD 卡中
set FEXTRAS /fs/microsd/etc/extras.txt  #定义额外启动脚本的路径，用于加载用户自定义模块
set FRC /fs/microsd/etc/rc.txt  #定义备用启动脚本路径，若此文件存在，通常会改变默认启动流程
set IOFW "/etc/extras/px4_io-v2_default.bin"  #指定PX4IO协同处理器的固件镜像路径，用于后续的固件版本校验或更新
```
- 系统状态与日志参数初始化
```
set LOGGER_ARGS ""  #初始化日志记录器（Logger）参数为空字符串,在脚本后期启动 logger 进程时，系统会将用户在 config.txt 或其他地方定义的额外参数累加到这个变量中。如果保持为空，则使用系统默认的日志配置
set LOGGER_BUF 8  #设置日志缓存大小，默认为8KB
set PARAM_FILE ""  #初始化主参数文件路径变量,这是一个占位符。在脚本后续检测到存储设备可用后，会将实际的参数文件路径赋值给它，随后通过param select命令进行加载 。
set PARAM_BACKUP_FILE "" #初始化参数备份文件路径变量,同样是占位符。当 SD 卡挂载成功后，该变量通常会被设置为 "/fs/microsd/parameters_backup.bson" 。这样当主参数文件损坏时，系统知道去哪里找备份进行恢复
set RC_INPUT_ARGS ""  #初始化遥控器输入（RC Input）启动参数,用于存储传递给 rc_input 驱动的特定指令 。例如，某些特殊的接收机协议需要额外的配置参数，这些参数会暂存在这里，直到执行 rc_input start $RC_INPUT_ARGS 时被调用
set STORAGE_AVAILABLE no  #初始化存储可用状态为no，后续挂载 SD 卡成功后会改为yes
set SDCARD_EXT_PATH /fs/microsd/ext_autostart  #指定SD卡上存放外部机架配置文件的文件夹路径
set SDCARD_FORMAT no  #初始化“是否格式化SD卡”标志为no
set STARTUP_TUNE 1  #置启动提示音类型，1 通常对应标准的系统启动音
set VEHICLE_TYPE none  #初始化机型类型为none，直到后续加载具体机架脚本（如多旋翼或固定翼）后才会改变
#------------------------------------------------------------------------------
set PARAM_DEFAULTS_VER 1  #设置机架参数默认版本号为1。如果开发者修改了机架默认参数并希望强制用户更新，会通过提高此版本号来触发参数重置

ver all  #这是一个可执行命令，调用后会在控制台打印当前系统的详细版本信息
```
---
  - 存储卡挂载:脚本首先通过检测块设备 /dev/mmcsd0 来尝试挂载 SD 卡到 /fs/microsd 路径 。如果挂载成功且没有发现 .format 文件，则将 STORAGE_AVAILABLE 变量设置为 yes 。若硬件设备不存在，则会尝试查询 MTD 参数分区作为替代方案
  - 挂载SD卡部分技术栈为FAT32
  ---
  - 尝试挂载SD卡
```
if [ -b "/dev/mmcsd0" ]
then
	if mount -t vfat /dev/mmcsd0 /fs/microsd
	then
		if [ -f "/fs/microsd/.format" ]
		then
			echo "INFO [init] format /dev/mmcsd0 requested (/fs/microsd/.format)"
			set SDCARD_FORMAT yes
			rm /fs/microsd/.format
			umount /fs/microsd

		else
			set STORAGE_AVAILABLE yes
		fi
	fi
```
- 执行格式化逻辑：如果初次挂载失败或检测到格式化标记，脚本会执行以下格式化并重新挂载的操作
```
  if [ $STORAGE_AVAILABLE = no -o $SDCARD_FORMAT = yes ]
	then
		echo "INFO [init] formatting /dev/mmcsd0"
		set STARTUP_TUNE 15 # tune 15 = SD_ERROR (overridden to SD_INIT if format + mount succeeds)

		if mkfatfs -F 32 /dev/mmcsd0
		then
			echo "INFO [init] card formatted"

			if mount -t vfat /dev/mmcsd0 /fs/microsd
			then
				set STORAGE_AVAILABLE yes
				set STARTUP_TUNE 14 # tune 14 = SD_INIT
			else
				echo "ERROR [init] card mount failed"
			fi
		else
			echo "ERROR [init] format failed"
		fi
	fi
```
- MTD备选方案与挂载逻辑结束：如果 /dev/mmcsd0 设备不存在，脚本会尝试通过 mft 查询其他可用的存储分区。
```
  else
	# Is there a device mounted for storage
	if mft query -q -k MTD -s MTD_PARAMETERS -v /mnt/microsd
	then
		set STORAGE_AVAILABLE yes
	fi
fi
```
- 异常处理：检查是否存在硬件故障(hardfault)产生的残留日志，如果存在崩溃日志，将启动提示音设为“错误音”(ERROR_TURN)来提醒用户，并且尝试将内存中的崩溃日志写入到SD卡中，方便后续导出分析。

```
if [ $STORAGE_AVAILABLE = yes ]
then
	if hardfault_log check
	then
		set STARTUP_TUNE 2 # tune 2 = ERROR_TUNE
		if hardfault_log commit
		then
			hardfault_log reset
		fi
	fi

	# Check for an update of the ext_autostart folder, and replace the old one with it
	if [ -e /fs/microsd/ext_autostart_new ]
	then
		echo "Updating external autostart files"
		rm -r $SDCARD_EXT_PATH
		mv /fs/microsd/ext_autostart_new $SDCARD_EXT_PATH
	fi

	set PARAM_FILE /fs/microsd/params
	set PARAM_BACKUP_FILE "/fs/microsd/parameters_backup.bson"
fi

```
- 参数加载与机架匹配
- 代码首先检查 SD 卡上是否存在一个名为`rc.txt`的完全替代脚本,如果有，脚本会直接执行它，并且跳过后面所有的默认启动逻辑，这通常用于底层调试，或者在文件系统损坏无法启动标准流程时进行救急
```
  if [ -f $FRC ]
then
    .$FRC
```

- 如果没有替代脚本，则加载`rc.filepaths`，获取系统中各种配置文件的标准路径。

```
else

	# Load param file location from kconfig #这里说从kconfig读取参数，其实就是我们裁剪固件的文件以及drivers和modules文件中的kconfig文件决定。这里解释了我们裁剪配置外设是怎么实现的。
	. ${R}etc/init.d/rc.filepaths  #这个文件就是编译固件时由构建系统生成的
```

- 确保存储在芯片内部 MTD 分区中的工厂校准数据（陀螺仪、加速度计的出厂校准值）是完好无损的，防止飞控加载错误的校准数据导致飞行事故
```
	# Check if /fs/mtd_params is a valid BSON file
	if ! bsondump docsize /fs/mtd_caldata
	then
		echo "New /fs/mtd_caldata size is:"
		bsondump docsize /fs/mtd_caldata
	fi
```

- 加载工厂校准数据
```
	#
	# Load parameters.
	#
	# if the board has a storage for (factory) calibration data
	if mft query -q -k MTD -s MTD_CALDATA -v /fs/mtd_caldata
	then
		param load /fs/mtd_caldata
	fi
```

- 加载用户主参数
```
	param select $PARAM_FILE
```

- 如果主参数文件损坏（导入失败），脚本会执行以下一系列急救措施：
- 在控制台打印错误信息
- 将开机提示音设置为错误音，用声音警告用户
```
	if ! param import
	then
		echo "ERROR [init] param import failed"
		set STARTUP_TUNE 2 # tune 2 = ERROR_TUNE
```

- 尝试打印损坏文件的结构以便调试
```
		bsondump $PARAM_FILE
```

- 如果 SD 卡可用，将损坏的参数文件复制一份保存，命名为 `param_import_fail.bson`，供开发者后续分析原因
```
		if [ -d "/fs/microsd" ]
		then
			# try to make a backup copy
			cp $PARAM_FILE /fs/microsd/param_import_fail.bson
```

- 尝试备份恢复,并且将内核启动日志输出到SD卡上的 `param_import_fail.txt` 文件中
- 这里是备份恢复时，是脚本在控制，脚本知道路径在哪，直接指给飞控看。
```
			# try importing from backup file
			if [ -f $PARAM_BACKUP_FILE ]
			then
				echo "[init] importing from parameter backup"

				# dump current backup file contents for comparison
				bsondump $PARAM_BACKUP_FILE

				param import $PARAM_BACKUP_FILE

				# overwrite invalid $PARAM_FILE with backup
				cp $PARAM_BACKUP_FILE $PARAM_FILE
			fi

			param status

			dmesg >> /fs/microsd/param_import_fail.txt &
		fi
	fi
```

- 如果SD卡可用，就告诉系统将参数的备份文件路径设置为 `$PARAM_BACKUP_FILE` ，之后保存参数的时候，也会在这个地址下保存一个备份。
- 这里的意思是说，在保存备份的时候是飞控系统在后台自动控制，所以需要提前注册好路径，飞控才知道往哪写。
```
	if [ $STORAGE_AVAILABLE = yes ]
	then
		param select-backup $PARAM_BACKUP_FILEv
	fi
```

- 以太网硬件检测与初始化（如果支持以太网的话）
```
	if mft query -q -k MFT -s MFT_ETHERNET -v 1
	then
		netman update -i eth0
	fi
```

- 在重置飞控大部分参数（如 PID、安全设置等）的同时，保留最关键的校准数据和机架配置，从而避免重置后必须重新进行繁琐的传感器和遥控器校准
```
	# To trigger a parameter reset during boot SYS_AUTCONFIG was set to 1 before
	if param greater SYS_AUTOCONFIG 0
	then
		# Reset parameters except airframe, parameter version, RC calibration, sensor calibration, flight modes, total flight time, flight UUID
```

- 这些参数在重置飞控时被保留
- `SYS_AUTOSTART`机架类型
- `SYS_PARAM_VER`参数版本号，用于防止重复触发重置
- `RC*`遥控器校准数据。保留了你的摇杆最大/最小值和通道映射，不需要重做 RC 校准
- `CAL_*`传感器校准数据。最关键的部分！保留了加速度计、陀螺仪、磁力计和水平校准数据
- `COM_FLTMODE*`飞行模式开关设置。保留了你习惯的“定点”、“自稳”等开关映射
- `LND_FLIGHT*` `TC_*` `COM_FLIGHT*`飞行统计数据。保留了总飞行时间、飞行次数和唯一的飞行 UUID，用于寿命记录
```
		param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* LND_FLIGHT* TC_* COM_FLIGHT*
	fi
```

- PX4 启动配置的分层加载机制中的第一层——架构级默认配置
- `PX4-Autopilot/platforms/nuttx/init/stm32h7/rc.board_arch_defaults`
- 在加载具体机型参数之前，先加载STM32H7架构通用的默认配置
- 将 EKF2（估计器）的多 IMU 融合模式设为 3（通常指多路并发融合或更激进的冗余策略）。
- 开启实时频谱分析 (FFT)
- 开启 PID 自动调参，开启此功能后，可以在QGC中绑定一个RC的按键，按下这个按键，STM32H7会故意向电机发送瞬间的扰动指令，让PX4内部的PID自动调参，获得更优化的参数。——系统辨识（System Identification）
- 开启高级 CAN 总线支持
- 增大日志缓存
```
	#
	# Optional board architecture defaults: rc.board_arch_defaults
	#
	set BOARD_ARCH_RC_DEFAULTS ${R}etc/init.d/rc.board_arch_defaults
	if [ -f $BOARD_ARCH_RC_DEFAULTS ]
	then
		echo "Board architecture defaults: ${BOARD_ARCH_RC_DEFAULTS}"
		. $BOARD_ARCH_RC_DEFAULTS
	fi
	unset BOARD_ARCH_RC_DEFAULTS
```
-板级硬件默认配置
-开网口: 默认配置好了以太网连接 QGC
-选芯片: 告诉系统电源模块是 INA226。
-管温度: 开启 IMU 加热
-定版本: 解决不同批次硬件的传感器 ID 冲突
-具体看下面这个文件
-文件位置`PX4-Autopilot/boards/ark/fmu-v6x/init/rc.board_defaults`
```

	#
	# Optional board defaults: rc.board_defaults
	#
	set BOARD_RC_DEFAULTS ${R}etc/init.d/rc.board_defaults
	if [ -f $BOARD_RC_DEFAULTS ]
	then
		echo "Board defaults: ${BOARD_RC_DEFAULTS}"
		. $BOARD_RC_DEFAULTS
	fi
	unset BOARD_RC_DEFAULTS
```
- 机架配置加载
- 根据设定的机架ID，从FLASH中找到对应的配置文件并运行
- 如果flash中没有会去sd卡中找
- 如果都没有则报错，会有蜂鸣器提示音
- `VEHICLE_TYPE`来自对应的机架文件
```
	# Load airframe configuration based on SYS_AUTOSTART parameter
	if ! param compare SYS_AUTOSTART 0
	then
		# rc.autostart directly run the right airframe script which sets the VEHICLE_TYPE
		# Look for airframe in ROMFS
		. ${R}etc/init.d/rc.autostart

		if [ ${VEHICLE_TYPE} == none ]
		then
			# Use external startup file
			if [ $STORAGE_AVAILABLE = yes ]
			then
				. ${R}etc/init.d/rc.autostart_ext
			else
				echo "ERROR [init] SD card not mounted - can't load external airframe"
			fi
		fi

		if [ ${VEHICLE_TYPE} == none ]
		then
			echo "ERROR [init] No airframe file found for SYS_AUTOSTART value"
			param set SYS_AUTOSTART 0
			tune_control play error
		fi
	fi
```
- 当刷写了新版本的固件时，强制系统进行一次重启和参数清理，以防止旧版本的参数在新固件上导致错误或炸机
- `SYS_AUTOCONFIG 1`，重新上电后会执行数据清除
```
	# Check parameter version and reset upon airframe configuration version mismatch.
	# Reboot required because "param reset_all" would reset all "param set" lines from airframe.
	if ! param compare SYS_PARAM_VER ${PARAM_DEFAULTS_VER}
	then
		echo "Switched to different parameter version. Resetting parameters."
		param set SYS_PARAM_VER ${PARAM_DEFAULTS_VER}
		param set SYS_AUTOCONFIG 1
		param save
		reboot
	fi
```
- 启动蜂鸣器驱动`tone_alarm`
```
	#
	# Start the tone_alarm driver.
	# Needs to be started after the parameters are loaded (for CBRK_BUZZER).
	#
	tone_alarm start
```
- 加载航点数据‘
- 如果`SYS_DM_BACKEND 1`，则从RAM中读取航点，快
- 如果`SYS_DM_BACKEND 0`，则从SD卡中读取航点，没那么快
- `dataman`即为`datamanager`
```
	#
	# Waypoint storage.
	# REBOOTWORK this needs to start in parallel.
	#
	if param compare -s SYS_DM_BACKEND 1
	then
		dataman start -r
	else
		if param compare SYS_DM_BACKEND 0
		then
			# dataman start default
			dataman start
		fi
	fi
```
- 启动事件发送器
- 外设 (Hardware) -> 驱动程序 (Driver) -> uORB (内部消息) -> send_event (广播员) -> MAVLink (QGC 弹窗) / 蜂鸣器 (滴滴响)
```
	#
	# Start the socket communication send_event handler.
	#
	send_event start
```
- 启动负载监控器
- `cpuload`
- 每隔一段时间检查一下`CPU、RAM`的占用率
```
	#
	# Start the resource load monitor.
	#
	load_mon start
```
- 启动状态指示灯
- `rgbled`通常指板载的PWM控制的LED
- 剩下的为特定I2C LED驱动芯片型号
```
	#
	# Start system state indicator.
	#
	rgbled start -X -q
	rgbled_ncp5623c start -X -q
	rgbled_lp5562 start -X -q
	rgbled_is31fl3195 start -X -q
```
- 加载用户自定义配置
- 允许在不重新编译固件的情况下，通过 SD 卡里的文件来修改飞控的启动行为
- 由这句`set FCONFIG /fs/microsd/etc/config.txt`设置了该文件
```
	#
	# Override parameters from user configuration file.
	#
	if [ -f $FCONFIG ]
	then
		echo "Custom: ${FCONFIG}"
		. $FCONFIG
	fi
```
- 启动传感器系统
- 通过判断`SYS_HITL`来区分是在进行HITL还是真实飞行
- 如果`SYS_HITL 0`，则为HITL仿真环境，意味着飞控不再读取真实的陀螺仪/加速度计、GPS数据，而是准备接收来自电脑（仿真器）发过来的“假数据”，这里启动的传感器都是模拟的传感器

```

	#
	# Sensors System (start before Commander so Preflight checks are properly run).
	#
	if param greater SYS_HITL 0
	then
		sensors start -h

		# disable GPS
		param set GPS_1_CONFIG 0

		# start the simulator in hardware if needed
		if param compare SYS_HITL 2
		then
			simulator_sih start
			sensor_baro_sim start
			sensor_mag_sim start
			sensor_gps_sim start
			sensor_agp_sim start
		fi
```
- 如果`SYS_HITL 1`，则为真实环境
- 先加载板载传感器配置，告诉系统传感器都在什么总线上
- 第二步加载通用传感器脚本，初始化一些标准的驱动逻辑
- 第三步启动电池监控，如果`BAT1_SOURCE 2`，则启动 esc_battery 驱动，从数字电调读取电压电流；如果`BAT1_SOURCE 1`，那么就启动`battery_status`。这是最常见的模拟电压电流计（ADC）驱动，也就是我们平时用的那种电源模块
- 第四步正式启动传感器，前面只是加载驱动，现在开始启动他们。
- 启动后台进程，开始读取所有传感器的数据，进行滤波、校准，然后发布到 uORB 总线上给姿态解算模块使用
```
	else
		#
		# board sensors: rc.sensors
		#
		set BOARD_RC_SENSORS ${R}etc/init.d/rc.board_sensors
		if [ -f $BOARD_RC_SENSORS ]
		then
			echo "Board sensors: ${BOARD_RC_SENSORS}"
			. $BOARD_RC_SENSORS
		fi
		unset BOARD_RC_SENSORS

		. ${R}etc/init.d/rc.sensors

		if param compare -s BAT1_SOURCE 2
		then
			esc_battery start
		fi

		if ! param compare BAT1_SOURCE 1
		then
			battery_status start
		fi

		sensors start
	fi
```
- 根据参数选择并启动状态估计器
- 飞控需要知道自己“在哪里”和“姿态如何”，这依靠估计器算法
- PX4 提供了三种算法
- EKF2（扩展卡尔曼滤波）--主流
- LPE（局部位置估计器）
- Attitude Q（Q 姿态估计器）
```
	#
	# state estimator selection
	#
	if param compare -s EKF2_EN 1
	then
		ekf2 start &
	fi

	if param compare -s LPE_EN 1
	then
		local_position_estimator start
	fi

	if param compare -s ATT_EN 1
	then
		attitude_estimator_q start
	fi
```
- PX4IO 协处理器（IO Co-processor）的固件检查、自动更新与启动
- 一些高级飞控采用双芯片架构，分别称为FMU(Flight Management Unit)和IO(Input/Output)
- FMU是主芯片，负责复杂的姿态解算、导航和通讯
- IO是协处理器专门负责输出 PWM 信号给电机、读取遥控器信号 (RC Input) 以及处理硬件安全开关（Safety Switch）
- 小飞控可能是单芯片，主控芯片负责所有功能
- 这段代码先判断是否由IO芯片，如果有，则检查IO芯片的固件和FMU的固件版本是否匹配，如果不匹配，FMU会刷新IO的固件，使版本匹配，最后启动IO，让其控制电机。
```
	#
	# px4io
	#
	if px4io supported
	then
	# Check if PX4IO present and update firmware if needed.
		if [ -f $IOFW ]
		then
			if ! px4io checkcrc ${IOFW}
			then
				# tune Program PX4IO
				tune_control play -t 16 # tune 16 = PROG_PX4IO

				if px4io update ${IOFW}
				then
					usleep 10000
					tune_control stop
					if px4io checkcrc ${IOFW}
					then
						tune_control play -t 17 # tune 17 = PROG_PX4IO_OK
					else
						tune_control play -t 18 # tune 18 = PROG_PX4IO_ERR
					fi
				else
					tune_control stop
				fi
			fi

			if ! px4io start
			then
				echo "PX4IO start failed"
				set STARTUP_TUNE 2 # tune 2 = ERROR_TUNE
			fi
		fi
	fi
```
- 启动 IMU（惯性测量单元）的恒温加热驱动
- 陀螺仪和加速度计对温度非常敏感
-
```
	# Heater driver for temperature regulated IMUs.
	# The heater needs to start after px4io.
	if param compare -s SENS_EN_THERMAL 1
	then
		heater start
	fi
```
- 处理遥控器（Remote Controller）输入的信号
- 发布`manual_control_setpoint`话题
```

	#
	# RC update (map raw RC input to calibrate manual control)
	#  start before commander
	#
	rc_update start
	manual_control start
```
- 启动相机控制、精准时间同步和转速测量
- 在飞控分配引脚给电机之前，先检查用户是否开启了航测拍照、时间同步或转速测量功能。如果有，就先把对应的引脚锁定给这些驱动使用，避免冲突。
```
	# Start camera trigger, capture and PPS before pwm_out as they might access
	# pwm pins
	if param greater -s TRIG_MODE 0
	then
		camera_trigger start
		camera_feedback start
	fi
	# PPS capture driver
	if param greater -s PPS_CAP_ENABLE 0
	then
		pps_capture start
	fi
	# RPM capture driver
	if param greater -s RPM_CAP_ENABLE 0
	then
		rpm_capture start
	fi
	# Camera capture driver
	if param greater -s CAM_CAP_FBACK 0
	then
		if camera_capture start
		then
			camera_capture on
		fi
	fi
```
- 启动指挥官和动力输出
- 让飞控开始处理飞行逻辑，并准备好控制电机
- `SYS_HITL 0`，即为HITL仿真模式
- `SYS_HITL 1`，即为真实飞行模式
```
	#
	# Commander
	#
	if param greater SYS_HITL 0
	then
		commander start -h

		if ! pwm_out_sim start -m hil
		then
			tune_control play error
		fi

	else
		commander start

		dshot start
		pwm_out start
	fi
```
- 启动飞行控制核心算法
- 根据`VEHICLE_TYPE`启动对应的飞行控制软件模块
- 如果是mc，它会启动`mc_att_control`（姿态控制器）、`mc_pos_control`（位置控制器）和 `mc_hover_thrust_estimator`（悬停油门估计器）
- 如果是fw，它会启动`fw_att_control`（固定翼姿态控制）和`fw_pos_control_l1`（L1 导航算法）
```
	#
	# Configure vehicle type specific parameters.
	# Note: rc.vehicle_setup is the entry point for all vehicle type specific setup.
	. ${R}etc/init.d/rc.vehicle_setup
```
- 罗盘偏差估计器
- 启动一个不需要转圈就能自动校准罗盘的高级功能
- 减少“罗盘受到干扰”的报错，提高航向的精准度，不需要每次飞之前都重新校准罗盘
```
	# Pre-takeoff continuous magnetometer calibration
	if param compare -s MBE_ENABLE 1
	then
		mag_bias_estimator start
	fi
```
- 板级专用 MAVLink 服务的自动加载
- 有些飞控的板子上会有需要MAVLink通信的外设，通过执行`rc.board_mavlink`文件，来配置这些外设的MAVLink通信
```
	#
	# Optional board mavlink streams: rc.board_mavlink
	#
	set BOARD_RC_MAVLINK ${R}etc/init.d/rc.board_mavlink
	if [ -f $BOARD_RC_MAVLINK ]
	then
		echo "Board mavlink: ${BOARD_RC_MAVLINK}"
		. $BOARD_RC_MAVLINK
	fi
	unset BOARD_RC_MAVLINK
```
- 启动串口驱动
- `rc.serial`根据在QGC中设置的参数，把飞控上的物理串口分配给对应的功能
```
	#
	# Start UART/Serial device drivers.
	# Note: rc.serial is auto-generated from Tools/serial/generate_config.py
	#
	. ${R}etc/init.d/rc.serial
```
- 遥控器的输入必须在串口配置完之后启动
- 因为先要确定哪些串口被占用了之后，用剩下的串口来扫描接收机信号，或者直接指定串口
```
	# Must be started after the serial config is read
	rc_input start $RC_INPUT_ARGS
```
- 管理USB接口
- 飞控上的USB在Nuttx系统里被识别为`ttyACM0`
- 每次将飞控连上电脑，是启动了一个MAVLink数据流，专门用来和地面站通信
```
	# Manages USB interface
	if param greater -s SYS_USB_AUTO -1
	then
		if ! cdcacm_autostart start
		then
			sercon
			echo "Starting MAVLink on /dev/ttyACM0"
			mavlink start -d /dev/ttyACM0
		fi
	fi
```
- 播放开机启动音效
- 如果启动失败，播放error音效
- `CBRK_BUZZER 782090`当把这个参数设为这个值时，禁用蜂鸣器
- 如果是报错音，会无视静音设置强制播放
```
	#
	# Play the startup tune (if not disabled or there is an error)
	#
	param compare CBRK_BUZZER 782090
	if [ "$?" != "0" -o "$STARTUP_TUNE" != "1" ]
	then
		tune_control play -t $STARTUP_TUNE
	fi
```
- 启动导航模块
- Commander -> Navigator -> Position Controller
```
	#
	# Start the navigator.
	#
	navigator start
```
- 检查并运行温度校准
- 只有在在QGC中修改参数才能使用该功能
```
	#
	# Start a thermal calibration if required.
	#
	set RC_THERMAL_CAL ${R}etc/init.d/rc.thermal_cal
	if [ -f ${RC_THERMAL_CAL} ]
	then
		. ${RC_THERMAL_CAL}
	fi
	unset RC_THERMAL_CAL
```
- 启动云台驱动
```
	#
	# Start gimbal to control mounts such as gimbals, disabled by default.
	#
	if param greater -s MNT_MODE_IN -1
	then
		gimbal start
	fi
```
- 黑羊遥测
- 老技术
```
	# Blacksheep telemetry
	if param compare -s TEL_BST_EN 1
	then
		bst start -X
	fi
```
- 重要!
- 陀螺仪 FFT 频谱分析
- 在飞行中实时分析陀螺仪数据里的频率
- 出来的频率数据会直接喂给动态陷波滤波器，滤掉噪声，减少电机发热等
```
	if param compare -s IMU_GYRO_FFT_EN 1
	then
		gyro_fft start
	fi
```
- 陀螺仪在线校准
- 启动运行时陀螺仪偏置校准
- 这不同于在地面站做的静态校准。这是一个在某些特定条件下运行的辅助校准逻辑，用于在长时间运行中修正漂移
```
	if param compare -s IMU_GYRO_CAL_EN 1
	then
		gyro_calibration start
	fi
```
- 检查PX4Flow 光流传感器
- 启动老款的 PX4Flow 光流模块驱动
- 用于室内无 GPS 环境下的定位
```
	# Check for px4flow sensor
	if param compare -s SENS_EN_PX4FLOW 1
	then
		px4flow start -X &
	fi
```
- 载荷投送
- 启动抛投器/夹爪控制模块
- 负责控制舵机打开钩子，或者控制磁铁断电扔下包裹
```
	payload_deliverer start
```
- 内燃机控制
- 启动油动发动机控制逻辑
- 用于油动无人机
```
	if param compare -s ICE_EN 1
	then
		internal_combustion_engine_control start
	fi
```
- 可选的板载附加组件
- 用于处理既不是传感器，也不是MAVLink通信的外设
- 比如说接一个OLED屏幕等等
- `rc.board_extras`扩展脚本
- 修改该PX4源码，需要重新编译和烧录
- 优点：稳定
```
	#
	# Optional board supplied extras: rc.board_extras
	#
	set BOARD_RC_EXTRAS ${R}etc/init.d/rc.board_extras
	if [ -f $BOARD_RC_EXTRAS ]
	then
		echo "Board extras: ${BOARD_RC_EXTRAS}"
		. $BOARD_RC_EXTRAS
	fi
	unset BOARD_RC_EXTRAS
```
- 加载SD卡上的自定义扩展脚本
- 启动那些官方固件里没有、但是自己写在 SD 卡里的程序
- 和`config.txt`有点区别
- 如果有扩展的外设，建议写在这个文件中，而不是上面的`rc.board_extras`
- 不需要重新编译，从SD卡中启动
- Nuttx Shell脚本，和rcS类似
- 缺点：SD故障会丢失
- 除非是很冷门的外设，一般都不需要写这个脚本，而是在QGC里面调参数来开启或者关闭外设
```
	#
	# Start any custom addons from the sdcard.
	#
	if [ -f $FEXTRAS ]
	then
		echo "Addons script: ${FEXTRAS}"
		. $FEXTRAS
	fi
```
- 启动日志系统
- "行车记录仪"
```

	#
	# Start the logger.
	#
	set RC_LOGGING ${R}etc/init.d/rc.logging
	if [ -f ${RC_LOGGING} ]
	then
		. ${RC_LOGGING}
	fi
	unset RC_LOGGING
```
- 这里也是机架配置
- 配置所有机架通用的配置
- 这些通用配置写在另外的文件里，而不是在每个机架文件里重复写
```
	#
	# Set additional parameters and env variables for selected AUTOSTART.
	#
	if ! param compare SYS_AUTOSTART 0
	then
		. ${R}etc/init.d/rc.autostart.post
	fi

```
- Bootloader 自动升级机制
- 检查当前的 PX4 固件里是否打包了新版本的 Bootloader，如果有就运行升级脚本来升级Bootloader
- Bootloader功能：检测USB有没有插着，如果有，就允许通过QGC刷入新固件；如果没有，就启动 PX4 固件
```
	set BOARD_BOOTLOADER_UPGRADE ${R}etc/init.d/rc.board_bootloader_upgrade
	if [ -f $BOARD_BOOTLOADER_UPGRADE ]
	then
		sh $BOARD_BOOTLOADER_UPGRADE
	fi
	unset BOARD_BOOTLOADER_UPGRADE
```
- 启动CAN总线
- 下面这两个二选一
- UAVCAN：旧版 UAVCAN v0
- Cyphal：新版 UAVCAN v1
- 是否开启Zenoh
- Zenoh ：网络协议，主要用于机器人领域（ROS 2）。它不仅仅是连电调的，更多是用来连接机载电脑、进行高吞吐量的云端通讯或者边缘计算通讯。它是独立的。不管你用不用 CAN，只要你想用 Zenoh 连网，它就会启动
```
	#
	# Check if UAVCAN is enabled, default to it for ESCs.
	#
	if param greater -s UAVCAN_ENABLE 0
	then
		# Start core UAVCAN module.
		if ! uavcan start
		then
			tune_control play error
		fi
	else
		if param greater -s CYPHAL_ENABLE 0
		then
			cyphal start
		fi
	fi
	if param greater -s ZENOH_ENABLE 0
	then
		zenoh start
	fi

#
# End of autostart.
#
fi
```
- 启动完成！


# 三.通信协议
- PX4内部通过uORB进行通信
- PX4和外界通过MAVLikn进行通信
## 1.uORB
- uORB 是 PX4 内部的异步消息传输机制（IPC，进程间通信）
- 传感器驱动只管发数据，不需要知道谁在用数据
- 工作方式：布/订阅模式 (Publish / Subscribe)
- Topic (话题)：从本质上讲，一个 Topic 就是一个结构体（Struct），定义在 .msg 文件中（例如 vehicle_attitude.msg）
- Node (节点)：任何一个后台进程（module）都可以是发布者（Advertiser）或订阅者（Subscriber）。

## 2.MAVLink
- MAVLink 是一种轻量级的通信协议，用于飞控与外部世界（地面站 QGC、机载电脑、OSD、数传）进行交互。
- 物理层：通常运行在 串口 (UART/Serial) 或 UDP/TCP 网络上。
- 逻辑层：它定义了一套消息 ID 和 Payload 格式。
- 飞控内部只认 uORB，外部只认 MAVLink。它们之间通过 mavlink 模块 进行转换
- ---
- 举例：
- ---
- 飞控 $\rightarrow$ 地面站 :
- mavlink 模块订阅 uORB 的 vehicle_attitude
- 收到更新后，打包成 MAVLink 的 ATTITUDE 消息包
- 通过串口发送出去
- ---
- 地面站 $\rightarrow$ 飞控 ：
- mavlink 模块从串口收到 COMMAND_LONG (比如起飞指令)
- 解析后，将其转换为 uORB 的 vehicle_command 消息并发布
- commander 模块订阅到该指令并执行

# 四.飞控算法
- 以 PX4 v1.16 多旋翼（MC）为例，主控制链可概括为：上层设定值 -> 位置控制 -> 姿态控制 -> 角速度控制 -> 控制分配 -> 执行器输出。
- 其中“串级控制”是更准确的表述：位置环负责生成姿态/推力设定值，姿态环基于四元数误差生成角速度设定值，角速度环使用 PID 生成力矩设定值，最后由 Control Allocation 将总力矩/总推力分配到各执行器。
- 上层输入不只来自遥控器（RC）或 Navigator，也可能来自 FlightTask、Mission、Offboard 或自定义控制模块。
- 位置控制（Outer Loop）：输入轨迹设定值（位置/速度/加速度/偏航等），输出姿态设定值与推力。
- 姿态控制（Middle Loop）：输入姿态设定值（四元数）与偏航速率设定值，结合当前姿态估计，输出角速度设定值。
- 角速度控制（Inner Loop）：输入角速度设定值，结合陀螺仪角速度反馈，经过 PID/FF/抗饱和等环节输出力矩设定值。
- 控制分配（Control Allocation）：输入力矩设定值/推力设定值，基于机架几何与执行器有效度矩阵进行分配，输出执行器控制量。
- 执行器输出层再根据配置通过 PWM、DShot、UAVCAN/Cyphal 等协议驱动 ESC/舵机。
- 下面按“内环 -> 姿态环 -> 外环 -> 控制分配 -> ESC 输出链路”说明。
## 1.角速度控制 (Inner Loop)
- **代码位置**：`src/modules/mc_rate_control`
- **主要职责**：输入角速度设定值，通过 PID 控制生成力矩设定值（Torque）。这是飞控中频率最高的闭环，直接影响姿态稳定性和响应速度。
- **订阅话题 (uORB Subscriptions)**：
    - `vehicle_rates_setpoint`: 来自姿态控制器的角速度设定值。
    - `vehicle_angular_velocity`: 经过滤波后的陀螺仪实时角速度。
    - `vehicle_control_mode`: 检查当前是否处于角速度控制使能状态。
- **核心算法**：
    - **PID 控制**：对 Roll/Pitch/Yaw 三轴分别进行 PID 运算。
	- **前馈 (Feed-Forward)**：利用角速度设定值直接给出一部分控制量，提升响应速度、减少滞后。
    - **抗饱和 (Anti-Windup)**：对积分环进行限幅，防止过度纠偏。
- **输出话题 (uORB Publications)**：
    - `vehicle_torque_setpoint`：发布力矩设定值；与 `vehicle_thrust_setpoint` 一起进入 Control Allocation，再由执行器输出层映射到电机/舵机。

### (1)MC角速度环控制参数
- **参数文件位置**：
	- `src/modules/mc_rate_control/mc_rate_control_params.c`
	- `src/modules/mc_rate_control/mc_acro_params.c`
- **基础PID增益参数**：
	- `MC_ROLLRATE_P/I/D`: 滚转轴 PID 增益。
	- `MC_PITCHRATE_P/I/D`: 俯仰轴 PID 增益。
	- `MC_YAWRATE_P/I/D`: 偏航轴 PID 增益。
- **理想式缩放与前馈参数**：
	- `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K`: 将并联 PID 形式缩放到理想形式的总增益。
	- `MC_ROLLRATE_FF` / `MC_PITCHRATE_FF` / `MC_YAWRATE_FF`: 角速度前馈增益。
- **积分限幅与抗饱和相关参数**：
	- `MC_RR_INT_LIM` / `MC_PR_INT_LIM` / `MC_YR_INT_LIM`: 三轴积分项限幅。
- **Acro 手动角速度映射参数（来自 `mc_acro_params.c`）**：
	- `MC_ACRO_R_MAX` / `MC_ACRO_P_MAX` / `MC_ACRO_Y_MAX`: Acro 模式三轴最大角速度。
	- `MC_ACRO_EXPO` / `MC_ACRO_EXPO_Y`: 摇杆 Expo 曲线（横滚俯仰/偏航）。
	- `MC_ACRO_SUPEXPO` / `MC_ACRO_SUPEXPOY`: 摇杆 SuperExpo 曲线（横滚俯仰/偏航）。
- **输出补偿与滤波参数**：
	- `MC_BAT_SCALE_EN`: 电池电压补偿开关（对推力/力矩设定值缩放）。
	- `MC_YAW_TQ_CUTOFF`: 偏航力矩输出低通截止频率（抑制高频抖动）。

### (2)MC角速度控制主循环
- **位置**：`src/modules/mc_rate_control/MulticopterRateControl.cpp`
- `init()`: 向调度器注册中断：只要陀螺仪更新数据，就立刻触发 `Run()` 函数。
```cpp
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}
```
- `parameters_updated()`:
- 从参数系统中读取 P,I,D,K,前馈 FF，积分限幅等数值。
- `_rate_control.setPidGains`, `_rate_control.setIntegratorLimit`, `_rate_control.setFeedForwardGain` 使用这些参数。
```cpp
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}
```

- `Run()`: 角速度环核心执行函数（由陀螺仪回调触发）
- 执行流程：取陀螺仪数据 -> 取/生成角速度设定值 -> PID 计算力矩 -> 发布 `vehicle_torque_setpoint` 与 `vehicle_thrust_setpoint`
```cpp
void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = angular_velocity.timestamp_sample;
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// ACRO: 摇杆直接生成角速度给定
			manual_control_setpoint_s manual_control_setpoint;
			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		if (_vehicle_control_mode.flag_control_rates_enabled) {
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
		}
	}

	perf_end(_loop_perf);
}
```

### (3)MC角速度环核心算法函数：三轴PID与前馈合成力矩
- **位置**：`src/lib/rate_control/rate_control.cpp`
- 执行流程：计算角速度误差 `rate_sp - rate` -> 按 P/I/D/FF 合成力矩 `torque` -> 非着陆状态更新积分项
```cpp
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	Vector3f rate_error = rate_sp - rate;
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}
```

## 2.姿态控制 (Middle Loop)
- **代码位置**：`src/modules/mc_att_control`
- **主要职责**：输入姿态设定值，输出角速度设定值。
- **订阅话题 (uORB Subscriptions)**：
    - `vehicle_attitude_setpoint`: 来自位置控制器或手动指令的姿态设定值（四元数）。
    - `vehicle_attitude`: 经过 EKF2 融合后的当前实际姿态。
- **核心算法**：
    - **四元数误差与减缩姿态（reduced attitude）处理**：先基于机体 z 轴方向构造减缩目标姿态，再与目标姿态组合处理偏航分量，兼顾推力方向优先与偏航跟随。
    - **比例映射到角速度设定值**：将姿态误差映射为角速度设定值，并结合偏航权重与角速度限幅输出 `vehicle_rates_setpoint`。
- **输出话题 (uORB Publications)**：
    - `vehicle_rates_setpoint`: 发布角速度设定值，直接送入下层角速度环。

### (1)MC姿态环控制参数
- **参数文件位置**：`src/modules/mc_att_control/mc_att_control_params.c`
- **核心增益参数**：
    - `MC_ROLL_P` / `MC_PITCH_P` / `MC_YAW_P`: 姿态误差到角速度设定值的比例增益。
    - `MC_YAW_WEIGHT`: 偏航权重，控制“姿态优先”与“偏航跟随”的折中。
- **角速度限制参数**：
    - `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX`: 姿态环输出角速度上限。
- **手动操纵相关参数**：
    - `MC_MAN_TILT_TAU`: 手动姿态输入一阶滤波时间常数。
    - `MPC_MAN_TILT_MAX`: 手动最大倾角。
    - `MPC_YAW_EXPO` / `MPC_HOLD_DZ`: 偏航杆输入非线性与死区。
    - `MPC_THR_CURVE` / `MPC_THR_HOVER` / `MPC_THR_MAX`: 油门曲线与悬停推力映射。

### (2)MC姿态控制主循环
- **位置**：`src/modules/mc_att_control/mc_att_control_main.cpp`
- 执行流程：订阅 `vehicle_attitude` / `vehicle_attitude_setpoint` -> 调用 `_attitude_control.update(q)` -> 发布 `vehicle_rates_setpoint`
```cpp
void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_attitude_sub.update(&v_att)) {
		const Quatf q{v_att.q};
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
					  && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)) {
					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
				}
			}

			Vector3f rates_sp = _attitude_control.update(q);

			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);
		}
	}
}
```

### (3)MC姿态环核心函数：四元数误差转角速度
- **位置**：`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`
- 执行流程：当前姿态 `q` 与目标姿态 `qd` 计算误差四元数 -> 提取虚部作为误差向量 -> 比例增益映射成 `rates_sp`
```cpp
matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		qd_red = qd;
	} else {
		qd_red *= q;
	}

	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	const Quatf qe = q.inversed() * qd;
	const Vector3f eq = 2.f * qe.canonical().imag();
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
```

## 3.位置控制 (Outer Loop)
- **代码位置**：`src/modules/mc_pos_control`
- **主要职责**：将轨迹设定值（位置/速度/加速度/偏航）转换为姿态设定值、推力以及局部位置设定值。
- **订阅话题 (uORB Subscriptions)**：
    - `trajectory_setpoint`：位置控制器的上游轨迹设定值输入，可来自 Navigator、FlightTask、Offboard 或自定义模块，并不局限于导航航点。
    - `vehicle_local_position`: 当前飞行器在局部坐标系下的位置和速度估计（来自 EKF2）。
- **核心算法**：
    - **分级控制**：位置误差通过 P 环生成速度设定值，速度误差通过 PID 生成加速度/推力相关设定值，再映射为姿态与推力输出。
    - **姿态映射**：将 XY 轴的加速度设定值映射为飞控的 Roll/Pitch 倾角，将 Z 轴加速度映射为总推力（Thrust）。
- **输出话题 (uORB Publications)**：
    - `vehicle_attitude_setpoint`: 发布姿态设定值（四元数）和推力。

### (1)MC位置环控制参数
- **参数文件位置**：
    - `src/modules/mc_pos_control/multicopter_position_control_gain_params.c`
    - `src/modules/mc_pos_control/multicopter_position_control_limits_params.c`
    - `src/modules/mc_pos_control/multicopter_takeoff_land_params.c`
    - `src/modules/mc_pos_control/multicopter_position_control_params.c`
- **核心控制增益参数**：
    - `MPC_XY_P` / `MPC_Z_P`: 位置 P 环增益。
    - `MPC_XY_VEL_P_ACC` / `MPC_XY_VEL_I_ACC` / `MPC_XY_VEL_D_ACC`: XY 速度 PID。
    - `MPC_Z_VEL_P_ACC` / `MPC_Z_VEL_I_ACC` / `MPC_Z_VEL_D_ACC`: Z 速度 PID。
- **约束与安全参数**：
    - `MPC_XY_VEL_MAX` / `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN`: 速度上限。
    - `MPC_THR_MIN` / `MPC_THR_MAX` / `MPC_THR_HOVER`: 推力上下限与悬停推力。
    - `MPC_TILTMAX_AIR` / `MPC_TILTMAX_LND`: 空中/起降最大倾角。
    - `MPC_ACC_HOR` / `MPC_ACC_UP_MAX` / `MPC_ACC_DOWN_MAX` / `MPC_JERK_MAX`: 加速度与 jerk 限制。
- **估计/滤波相关参数**：
    - `MPC_USE_HTE`: 是否使用悬停推力估计器。
    - `MPC_VEL_LP` / `MPC_VELD_LP` / `MPC_VEL_NF_FRQ` / `MPC_VEL_NF_BW`: 速度与速度导数滤波参数。

### (2)MC位置控制主循环
- **位置**：`src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- 执行流程：订阅状态与轨迹设定值 -> 调用 `_control.update(dt)` -> 发布 `vehicle_local_position_setpoint` 与 `vehicle_attitude_setpoint`
```cpp
void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt = math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};
		_trajectory_setpoint_sub.update(&_setpoint);
		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			_control.setInputSetpoint(_setpoint);
			_control.setState(states);

			if (_control.update(dt)) {
				_last_valid_setpoint = _setpoint;
			}

			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			_local_pos_sp_pub.publish(local_pos_sp);

			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
		}
	}
}
```

### (3)MC位置环核心函数：位置P + 速度PID + 推力姿态映射
- **位置**：`src/modules/mc_pos_control/PositionControl/PositionControl.cpp`
```cpp
bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	}

	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::setZeroIfNanVector3f(vel_sp_position);
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}
```

## 4.控制分配（Control Allocation）
- **主要职责**：将控制环输出的“总力矩 + 总推力”分配到各个执行器通道。
- **订阅话题**：`vehicle_torque_setpoint`、`vehicle_thrust_setpoint`
- **输出话题**：`actuator_motors` / `actuator_servos` 等执行器控制量话题；其后再由具体输出驱动转换为 PWM、DShot、UAVCAN/Cyphal 等协议。
- **逻辑**：根据机架几何、执行器布局和 effectiveness matrix（有效度矩阵）进行控制分配。

### (1)控制分配参数
- **参数定义位置**：`src/modules/control_allocator/module.yaml`
- **核心参数（方法与几何）**：
    - `CA_AIRFRAME`: 机架类型（Multirotor / FW / VTOL 等）。
    - `CA_METHOD`: 分配算法（伪逆裁剪 / 顺序反饱和 / 自动）。
- **执行器约束参数**：
    - `CA_R_REV`: 可逆电机配置。
    - `CA_R${i}_SLEW` / `CA_SV${i}_SLEW`: 电机/舵机斜率限制。
- **几何与气动参数（多旋翼）**：
    - `CA_ROTOR_COUNT`: 桨数量。
    - `CA_ROTOR${i}_PX/PY/PZ`: 旋翼位置。
    - `CA_ROTOR${i}_AX/AY/AZ`: 旋翼推力方向。
    - `CA_ROTOR${i}_CT/KM`: 推力系数与反扭矩系数。
    - `CA_ROTOR${i}_TILT`: 倾转绑定。

### (2)控制分配主循环
- **位置**：`src/modules/control_allocator/ControlAllocator.cpp`
- 执行流程：订阅 `vehicle_torque_setpoint` / `vehicle_thrust_setpoint` -> 组装控制向量 `c` -> 调用 `allocate()` 分配到各执行器 -> 发布 `actuator_motors`
```cpp
void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);
		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;
	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
	}

	if (do_update) {
		_last_run = now;
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setControlSetpoint(c[i]);
			_control_allocation[i]->allocate();
			_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	publish_actuator_controls();
}
```

### (3)控制分配核心算法函数（伪逆分配）
- **位置**：`src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`
- 该函数的作用是：根据控制向量 `c`（总力矩/总推力）和执行器有效度矩阵的伪逆，求解执行器控制向量 `u`；其中 `u` 是分配后的各执行器控制量，后续仍需经过限幅、斜率限制以及具体输出驱动层处理。
```cpp
void
ControlAllocationPseudoInverse::allocate()
{
	// Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
```

- 说明：
- `_mix` 来自有效度矩阵（effectiveness matrix）的伪逆，反映机架几何和执行器布局。
- `_control_sp` 是力矩设定值/推力设定值，`_actuator_sp` 是最终每个电机（或舵机）输出。
- 之后还会经过 `applySlewRateLimit()` 与 `clipActuatorSetpoint()`，用于限制输出变化速度并避免超限。

## 5.ESC 控制链路（执行器输出层）

- **定位**：这一层位于控制分配之后，负责把 `actuator_motors` / `actuator_servos` 等执行器控制量转换成具体硬件输出协议。
- **与控制部分的联系**：
    - 控制算法层负责产生总力矩、总推力与执行器控制量；
    - 输出层负责把这些控制量送到 ESC/舵机，不同协议只影响输出封装与传输方式，不改变上层控制律本身。
- **常见输出协议**：
    - PWM：传统脉宽输出，兼容性高；
    - DShot：数字电调协议，抗干扰更好，并可扩展到双向 DShot/ESC telemetry；
    - UAVCAN/Cyphal：通过总线连接智能 ESC/外设。

# 五.感知导航

## 1.EKF2（扩展卡尔曼滤波）
- **代码位置**：`src/modules/ekf2`
- **主要职责**：融合 IMU、GNSS、气压计、磁罗盘、视觉里程计、光流、测距等多源观测，输出控制、导航与状态监测所需的姿态、位置、速度及估计器诊断信息。
- **处理流程**：传感器采样入队（带延迟补偿） $\rightarrow$ 状态与协方差预测 $\rightarrow$ 各观测源融合调度 $\rightarrow$ 发布姿态/位置/状态估计话题。
- **关键输入话题（uORB）**：
    - `vehicle_imu`：EKF2 的主惯导输入；
    - `vehicle_gps_position`：GNSS 位置/速度/高度观测。
    - `vehicle_air_data`：气压高度观测。
    - `vehicle_magnetometer`：磁场与航向相关观测。
    - `vehicle_visual_odometry`：外部视觉位置/速度/偏航观测。
    - `vehicle_optical_flow`、`distance_sensor`：近地辅助观测。
- **关键输出话题（uORB）**：
    - **控制面核心输出**：
        - `vehicle_attitude`
        - `vehicle_local_position`
        - `vehicle_global_position`
        - `vehicle_odometry`
    - **诊断与调参输出**：
        - `estimator_status`
        - `estimator_status_flags`
        - `estimator_states`
        - `estimator_sensor_bias`
        - （可选）`wind`、`yaw_estimator_status`

### (1)EKF2参数分组（按功能）
- **参数文件位置**：
    - `src/modules/ekf2/module.yaml`
    - `src/modules/ekf2/params_gnss.yaml`
    - `src/modules/ekf2/params_magnetometer.yaml`
    - `src/modules/ekf2/params_external_vision.yaml`
    - `src/modules/ekf2/params_barometer.yaml`
    - `src/modules/ekf2/params_range_finder.yaml`
    - `src/modules/ekf2/params_optical_flow.yaml`
    - `src/modules/ekf2/params_multi.yaml`

- **基础时序与滤波框架参数**（决定延迟补偿、输出平滑与 IMU 过程噪声建模）：
    - `EKF2_DELAY_MAX`：最大观测延迟窗口。
    - `EKF2_HGT_REF`：高度参考源（气压/GPS/测距/视觉）。
    - `EKF2_TAU_POS` / `EKF2_TAU_VEL`：输出预测器位置/速度时间常数。
    - `EKF2_IMU_CTRL`、`EKF2_GYR_NOISE`、`EKF2_ACC_NOISE`：IMU 融合控制与过程噪声。

- **GNSS 融合参数**（决定 GPS 什么时候可用、怎么融合）：
    - `EKF2_GPS_CTRL`：经纬度/高度/速度/双天线航向融合开关。
    - `EKF2_GPS_DELAY`：GNSS 延迟补偿。
    - `EKF2_GPS_P_NOISE` / `EKF2_GPS_V_NOISE`：位置/速度观测噪声。
    - `EKF2_GPS_P_GATE` / `EKF2_GPS_V_GATE`：创新门限。
    - `EKF2_GPS_CHECK` + `EKF2_REQ_NSATS/EPH/EPV/SACC`：健康检查与准入门槛。

- **磁罗盘与航向参数**（决定偏航稳定性与抗磁干扰能力）：
    - `EKF2_MAG_TYPE`：磁融合模式（自动/仅航向/禁用等）。
    - `EKF2_MAG_DELAY`、`EKF2_MAG_NOISE`、`EKF2_MAG_GATE`：延迟、噪声、门限。
    - `EKF2_DECL_TYPE`：磁偏角处理策略。
    - `EKF2_MAG_CHECK`、`EKF2_MAG_CHK_STR`、`EKF2_MAG_CHK_INC`：磁场强度/倾角一致性检查。

- **高度源参数（气压/测距/视觉）**：
    - 气压：`EKF2_BARO_CTRL`、`EKF2_BARO_DELAY`、`EKF2_BARO_NOISE`、`EKF2_BARO_GATE`。
    - 测距：`EKF2_RNG_CTRL`、`EKF2_RNG_DELAY`、`EKF2_RNG_NOISE`、`EKF2_RNG_A_VMAX/HMAX`。
    - 视觉：`EKF2_EV_CTRL`、`EKF2_EV_DELAY`、`EKF2_EVP_NOISE`、`EKF2_EVV_NOISE`。

- **近地与多实例参数**：
    - 光流：`EKF2_OF_CTRL`、`EKF2_OF_DELAY`、`EKF2_OF_N_MIN/MAX`、`EKF2_OF_QMIN`。
    - 多实例：`EKF2_MULTI_IMU`、`EKF2_MULTI_MAG`。

### (2)EKF2主循环
- **位置**：`src/modules/ekf2/EKF2.cpp`
- 处理流程：参数更新与话题初始化 -> 注入 IMU 样本并更新输出预测器 -> 更新各观测样本（Baro/EV/Flow/GPS/Mag/Range） -> 调用 `_ekf.update()` 完成预测与融合 -> 发布姿态、位置与诊断状态。
```cpp
void EKF2::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_imu_sub.unregisterCallback();
		return;
	}

	if (_parameter_update_sub.updated() || !_callback_registered) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
		VerifyParams();
		AdvertiseTopics();
		_ekf.updateParameters();
	}

	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us;
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now);

		UpdateBaroSample(ekf2_timestamps);
		UpdateExtVisionSample(ekf2_timestamps);
		UpdateFlowSample(ekf2_timestamps);
		UpdateGpsSample(ekf2_timestamps);
		UpdateMagSample(ekf2_timestamps);
		UpdateRangeSample(ekf2_timestamps);

		if (_ekf.update()) {
			PublishLocalPosition(now);
			PublishOdometry(now, imu_sample_new);
			PublishGlobalPosition(now);
			PublishSensorBias(now);
			PublishStatus(now);
			PublishStatusFlags(now);
		}
	}
}
```

### (3)EKF2核心算法函数

#### 3.1 预测：状态 + 协方差
- **位置**：`src/modules/ekf2/EKF/ekf.cpp`、`src/modules/ekf2/EKF/covariance.cpp`
- 主线：在延迟对齐后的 IMU 样本上执行预测与融合调度；当前代码路径中先进行协方差预测 `predictCovariance()`，再进行状态预测 `predictState()`，随后调用 `controlFusionModes()` 决定各观测源的融合策略。
```cpp
bool Ekf::update()
{
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	if (_imu_updated) {
		_imu_updated = false;
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);
		controlFusionModes(imu_sample_delayed);

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _gpos,
					      _state.gyro_bias, _state.accel_bias);
		return true;
	}

	return false;
}
```

#### 3.2 融合调度：多传感器门控与回退
- **位置**：`src/modules/ekf2/EKF/control.cpp`、`src/modules/ekf2/EKF/height_control.cpp`
- 主线：按控制状态与观测可用性，决定融合哪些传感器；高度源失效时自动回退到备选参考源。
```cpp
void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	_control_status_prev.value = _control_status.value;

	if (!_control_status.flags.tilt_align) {
		if (getTiltVariance() < sq(math::radians(3.f))) {
			_control_status.flags.tilt_align = true;
		}
	}

	controlMagFusion(imu_delayed);
	controlOpticalFlowFusion(imu_delayed);
	controlGpsFusion(imu_delayed);
	controlHeightFusion(imu_delayed);
	controlExternalVisionFusion(imu_delayed);

	controlFakePosFusion();
	controlFakeHgtFusion();
	updateDeadReckoningStatus();
}
```

#### 3.3 观测接口：时间对齐与延迟补偿
- **位置**：`src/modules/ekf2/EKF/estimator_interface.cpp`
- 主线：IMU 下采样后推入延迟缓冲区；GNSS/磁罗盘/气压等观测按各自 delay 参数回推时间戳，保证在同一融合时域内计算。
```cpp
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	_time_latest_us = imu_sample.time_us;
	_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang, imu_sample.delta_ang_dt,
					imu_sample.delta_vel, imu_sample.delta_vel_dt);

	if (_imu_down_sampler.update(imu_sample)) {
		_imu_updated = true;
		imuSample imu_downsampled = _imu_down_sampler.getDownSampledImuAndTriggerReset();
		_imu_buffer.push(imu_downsampled);
		_time_delayed_us = _imu_buffer.get_oldest().time_us;
		_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
	}
}

void EstimatorInterface::setGpsData(const gnssSample &gnss_sample)
{
	const int64_t time_us = gnss_sample.time_us
				- static_cast<int64_t>(_params.gps_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	if (time_us >= static_cast<int64_t>(_gps_buffer->get_newest().time_us + _min_obs_interval_us)) {
		gnssSample gnss_sample_new(gnss_sample);
		gnss_sample_new.time_us = time_us;
		_gps_buffer->push(gnss_sample_new);
	}
}
```
# 六.mavlink消息压缩

## 1.目标
- 目标：在 PX4 v1.16 + QGC 5.0.8 软件下，对数据链进行“可控压缩”，使其能够支持一站多机（10 架飞行器以上，目标支持 30 架飞行器并发接入）。
- 原则：优先保证基本的“监控、控制、告警”功能，其次才是高频调试可视化。
- 范围：本节同时覆盖
	- 监控链路（用于获取状态，包括 QGC 状态监控和位置监控）
	- Offboard链路（用于外部控制/算法接口）
- 详细改动见ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink与Tools/simulation/gazebo-classic/sitl_multiple_run.sh:21-28

---

## 2.总体思路

### (1)修改模式
- 将遥测链路优先切换为 `minimal` 或 `custom`。
- 建议优先 `custom`：默认流最少，便于做白名单控制。

### (2)限制最大速率
- 为每条 MAVLink 实例设置总发送上限（`mavlink start -r` 或 `MAV_X_RATE`）。
- 作用：防止某些流被误开高频后挤爆链路。

### (3)降低频率或关闭
- 只保留“任务必须消息”，其余降频或关闭（`mavlink stream -s <STREAM> -r <Hz>`，`0` 表示关闭）。
- 这是多机场景中最关键的压缩层。

---

## 3.消息压缩步骤

### Step 1：采集未压缩时的带宽
- 在飞控侧执行 `mavlink status`，记录各实例状态与 `rate mult`。
- 在 QGC 的 MAVLink Inspector 记录主要消息频率（至少包含 `GLOBAL_POSITION_INT`、`HEARTBEAT`、`ATTITUDE`、`LOCAL_POSITION_NED`）。

### Step 2：修改模式，限制总速率
- 将主监控链路改为 `minimal/custom`。
- 配置实例总速率上限（按链路逐步下压，出现丢关键信息再小幅回调）。

### Step 3：选择保留的消息
- 保留必要监控消息低频输出。
- Offboard 链路只保留控制闭环需要的消息，不复用监控链路的高频姿态可视化流。

### Step 4：测试
- 1 架 -> 5 架 -> 10 架 -> 20 架 -> 30 架逐级压测。
- 每级检查：
	- 关键消息是否还在目标频率范围内
	- `rate mult` 是否长期显著低于 1
	- 是否出现心跳中断、位置卡顿、Offboard 超时

### Step 5：保存配置
- 将最终稳定方案固化到启动脚本/参数配置，避免上电后回退默认高频流。

---

## 4.消息分级清单

### A.需要保留的消息（核心消息）

| 消息/流名 | 建议频率 | 用途 |
|---|---:|---|
| `HEARTBEAT` | 1 Hz | 链路存活、模式与基本状态 |
| `GLOBAL_POSITION_INT` | 1~2 Hz | 地图位置监控核心 |
| `SYS_STATUS` | 0.5~1 Hz | 系统健康、电源概要 |
| `BATTERY_STATUS` | 0.5~1 Hz | 电量监控 |
| `EXTENDED_SYS_STATE` | 0.2~1 Hz | 落地/起飞等扩展状态 |

可选保留：
- `GPS_RAW_INT`：0.5~1 Hz（需要定位质量时保留）
- `HOME_POSITION`：0.05~0.2 Hz（需要返航点显示时保留）
- `STATUSTEXT`：事件触发（不主动高频推送）

### B.降低频率的消息（按需启用）

| 消息/流名 | 建议频率 | 场景 |
|---|---:|---|
| `LOCAL_POSITION_NED` | 0~1 Hz | 仅在本地坐标监控需要时开启 |
| `RC_CHANNELS` | 0~1 Hz | 仅在排查遥控输入问题时开启 |
| `POSITION_TARGET_LOCAL_NED` | 0~1 Hz | 仅在调试目标跟踪时开启 |

### C.需要关闭的消息（默认关闭项）

| 消息/流名 | 建议 | 原因 |
|---|---|---|
| `ATTITUDE` | 关闭 | 姿态球类高频可视化，多机场景耗流大 |
| `ATTITUDE_QUATERNION` | 关闭 | 与姿态可视化相关，非核心监控 |
| `ATTITUDE_TARGET` | 关闭 | 目标姿态调试流，非必要 |
| `SERVO_OUTPUT_RAW_0` | 关闭 | 执行器调试流，非必要 |
| `OPTICAL_FLOW_RAD` | 关闭 | 专项传感器调试流，非必要 |
| `HIGHRES_IMU` | 关闭 | 高频传感器流，链路成本高 |

---

## 5.Offboard 模式相关

- 监控链路与 Offboard 链路建议逻辑隔离：
	- 监控链路：只保留低频消息
	- Offboard 链路：只保留控制必须消息
- 多机默认端口分配在 10 架以上会出现复用风险（尤其 Offboard 远端口范围 14540~14549，超过后会复用）。
- 目标支持 30 架飞行器并发接入时，建议：
	- 使用路由/汇聚（如 mavlink-router）统一管理转发
	- 确保 SYSID 唯一、端口策略明确
	- 避免调试高频流混入 Offboard 关键控制链路

---

## 6.修改示例

```sh
# 1) 启动低带宽实例（示例）
mavlink start -u <LOCAL_PORT> -o <REMOTE_PORT> -m custom -r <RATE_BPS>

# 2) 仅保留核心监控流（示例）
mavlink stream -u <LOCAL_PORT> -s HEARTBEAT -r 1
mavlink stream -u <LOCAL_PORT> -s GLOBAL_POSITION_INT -r 1
mavlink stream -u <LOCAL_PORT> -s SYS_STATUS -r 1
mavlink stream -u <LOCAL_PORT> -s BATTERY_STATUS -r 1
mavlink stream -u <LOCAL_PORT> -s EXTENDED_SYS_STATE -r 1

# 3) 关闭高耗流（示例）
mavlink stream -u <LOCAL_PORT> -s ATTITUDE -r 0
mavlink stream -u <LOCAL_PORT> -s ATTITUDE_QUATERNION -r 0
mavlink stream -u <LOCAL_PORT> -s ATTITUDE_TARGET -r 0
mavlink stream -u <LOCAL_PORT> -s LOCAL_POSITION_NED -r 0
mavlink stream -u <LOCAL_PORT> -s SERVO_OUTPUT_RAW_0 -r 0
mavlink stream -u <LOCAL_PORT> -s RC_CHANNELS -r 0
mavlink stream -u <LOCAL_PORT> -s OPTICAL_FLOW_RAD -r 0
```


---

# 7.EKF2的参数及流程梳理

## 7.1 目标与使用方式
- 本章目标：为后续传感器融合调参提供可直接执行的流程和关键参数入口。
- 本章重点：调参顺序、关键参数及异常排查方法。
- 适用版本：PX4 v1.16（以当前仓库参数定义与源码实现为准）。
- 注：本章提到的“杆臂”是指传感器到飞行器参考点（通常是机体质心或 IMU 参考点）的空间偏移量

---

## 7.2 EKF2主流程（调参视角）

### (1)主循环处理流程
- IMU 更新触发 EKF2 主循环。
- 主循环中先注入 IMU 样本，再更新并注入其他观测（GPS/Baro/Mag/EV/Flow/Range 等）。
- 执行 `_ekf.update()` 进行预测与融合。
- 更新成功后发布姿态、局部位置、全局位置、里程计与状态诊断话题。

### (2)为什么调参要先看“时间”
- EKF2 并不是把所有观测都在“当前时刻”直接融合，而是在延迟融合时域内按时间对齐融合。
- 因此 `EKF2_*_DELAY`、`EKF2_DELAY_MAX`、`EKF2_TAU_POS/VEL` 会直接影响动态机动时的飞行表现，如滞后、超调和抖动。

---

## 7.3 调参顺序

### A.安装与杆臂（先校几何）
- 先确认 IMU、GPS、EV、Flow、Range 的安装位置/方向是否与参数一致。
- 代表参数：
	- `EKF2_IMU_POS_X/Y/Z`
	- `EKF2_GPS_POS_X/Y/Z`
	- `EKF2_EV_POS_X/Y/Z`
	- `EKF2_OF_POS_X/Y/Z`
	- `EKF2_RNG_POS_X/Y/Z`、`EKF2_RNG_PITCH`

### B.噪声参数（再定权重）
- 噪声越小，EKF越“相信”该观测；噪声越大，EKF越“保守”。
- 代表参数：
	- `EKF2_GYR_NOISE`、`EKF2_ACC_NOISE`
	- `EKF2_GPS_P_NOISE`、`EKF2_GPS_V_NOISE`
	- `EKF2_BARO_NOISE`、`EKF2_MAG_NOISE`
	- `EKF2_EVP_NOISE`、`EKF2_EVV_NOISE`、`EKF2_EVA_NOISE`

### C.创新门限（再调拒收敏感度）
- `*_GATE` 过小：容易拒收有效观测；过大：容易接纳异常观测。
- 代表参数：
	- `EKF2_GPS_P_GATE`、`EKF2_GPS_V_GATE`
	- `EKF2_BARO_GATE`、`EKF2_MAG_GATE`
	- `EKF2_EVP_GATE`、`EKF2_EVV_GATE`
	- `EKF2_OF_GATE`、`EKF2_RNG_GATE`

### D.延迟补偿（最后调动态一致性）
- 动态机动时出现“相位滞后/跟随慢/回正慢”，优先检查延迟参数。
- 代表参数：
	- `EKF2_GPS_DELAY`、`EKF2_EV_DELAY`、`EKF2_OF_DELAY`
	- `EKF2_MAG_DELAY`、`EKF2_BARO_DELAY`、`EKF2_RNG_DELAY`
	- `EKF2_DELAY_MAX`

---

## 7.4 核心参数

> 说明：默认值以 `src/modules/ekf2/*.yaml` 为准；若编译时关闭对应功能，参数可能不会出现在固件里。

| 参数 | 默认值 | 类别 | 对应问题 | 调整方向 | 风险 |
|---|---:|---|---|---|---|
| `EKF2_IMU_POS_X/Y/Z` | 0/0/0 m | 杆臂 | 急加减速时姿态/位置耦合异常 | 按实测几何填写 | 中 |
| `EKF2_GPS_POS_X/Y/Z` | 0/0/0 m | 杆臂 | 转弯时位置偏、航向耦合 | 按天线相对质心填写 | 中 |
| `EKF2_EV_POS_X/Y/Z` | 0/0/0 m | 杆臂 | 室内位置控制发散或慢漂 | 按视觉传感器安装点填写 | 中 |
| `EKF2_GYR_NOISE` | 0.015 rad/s | 噪声 | 姿态估计过硬/过松 | 振动大可小幅增大 | 中 |
| `EKF2_ACC_NOISE` | 0.35 m/s² | 噪声 | 速度/高度创新异常 | 振动场景适度增大 | 中 |
| `EKF2_GPS_P_NOISE` | 0.5 m | 噪声 | GPS跳变影响位置稳定 | 干扰大时适度增大 | 中 |
| `EKF2_GPS_V_NOISE` | 0.3 m/s | 噪声 | 速度约束弱或过敏感 | 按GNSS质量微调 | 中 |
| `EKF2_BARO_NOISE` | 3.5 m | 噪声 | 高度抖动或慢漂 | 动压扰动大时适度增大 | 中 |
| `EKF2_MAG_NOISE` | 0.05 gauss | 噪声 | 偏航抖动或易受磁干扰 | 干扰环境适度增大 | 中 |
| `EKF2_GPS_P_GATE/V_GATE` | 5 / 5 SD | 门限 | GPS频繁拒收或误收 | 先小步调整，不要大跳 | 高 |
| `EKF2_BARO_GATE` | 5 SD | 门限 | 高度观测频繁拒收 | 与 BARO_NOISE 联调 | 中 |
| `EKF2_MAG_GATE` | 3 SD | 门限 | 偏航创新异常 | 先查磁环境再调门限 | 高 |
| `EKF2_EVP_GATE/EVV_GATE` | 5 / 3 SD | 门限 | EV 偶发跳变导致拒绝融合 | 与 EV 协方差联调 | 高 |
| `EKF2_GPS_DELAY` | 110 ms | 延迟 | 急机动时位置跟随滞后 | 按日志相位差微调 | 高 |
| `EKF2_EV_DELAY` | 0 ms | 延迟 | EV融合后漂移/发散 | 先做时钟同步再调 | 高 |
| `EKF2_OF_DELAY` | 20 ms | 延迟 | 低空速度估计相位错位 | 结合光流时间戳微调 | 中 |
| `EKF2_DELAY_MAX` | 200 ms | 延迟 | 某些观测延迟补偿失效 | 保证 ≥ 各类 delay 最大值 | 高 |

---


## 7.5 推荐调参流程

1. **提取当前飞行日志**：记录当前参数、日志、典型动作（悬停/加速/减速/转向）。
2. **先校准几何关系，再调整观测权重**：先改杆臂与安装偏置，再改噪声。
3. **再调门限**：只在确认观测质量后调 `*_GATE`。
4. **最后调延迟**：通过动态动作对齐相位，微调 `*_DELAY`。
5. **一次只改一组参数**：每次改动后必须复飞验证，避免参数耦合误判。
6. **记录参数修改历史**：每轮保存参数快照，异常可快速回滚。

---

## 7.6 编译剪裁相关

- EKF2 的功能开关在 `src/modules/ekf2/Kconfig`。
- 编译装配在 `src/modules/ekf2/CMakeLists.txt`：
	- 关闭某个 `CONFIG_EKF2_*` 后，不仅对应融合源码会被裁剪；
	- 相关 `params_*.yaml` 也可能不会进入固件参数集。
- 结果：QGC 上可能看不到对应参数；日志里也可能缺少相关诊断项。
- 因此建议：每次做 EKF2 剪裁后，先核对参数是否存在，再开始飞行调参。

---

## 7.8 与第5章的区别
- 第5章：偏“模块逻辑与源码路径”的系统解析。
- 第7章：偏“后续融合调参”的操作手册与排障路径。
- 使用建议：先按第5章理解数据流，再按第7章执行调参与回归。

# 8.自定义算法模板

## 8.1 目标与定位
- 需求：整理一套可复用的 PX4 v1.16 自定义算法模板，方便后续算法改进与编队研发。
- 定位：给出可直接落地的工程模板。
- 适用：控制类、非阻塞、周期性/事件驱动模块，适合编队控制、协同控制、轨迹层算法。

---

## 8.2 为什么优先用工作队列（Work Queue, WQ）模板

### (1)PX4自定义功能方式
- 任务（Task）：
任务是独立运行的模块，具有自己的堆栈和进程优先级。其模板位于src/templates/template_module，可用来创建新的任务模块。任务模块适合需要独立运行、长时间计算或需要阻塞 I/O 的场景。
- 工作队列（Work Queue Task）：
工作队列任务在共享的工作队列线程上运行，与其他任务共享堆栈和线程优先级。其模板位于src/examples/work_item，可用来创建工作队列任务模块。工作队列能减少内存占用和任务切换的开销，适合需要高效利用资源的场景
- 任务适合独立运行的模块，而工作队列任务更节省资源，但不支持阻塞 I/O 或消息轮询。

### (2)工作队列优势
- 线程共享：相比独立后台线程，更省 RAM、上下文切换更少。
- 调度域可控：可按队列选择优先级域（如 `nav_and_controllers`、`rate_ctrl`）。
- 与 uORB 回调机制天然适配：可通过订阅更新事件触发 `Run()`，减少无效轮询。

### (3)注意
- WQ 模块不能做阻塞行为：不要在 `Run()` 里 `sleep/poll/阻塞IO`。
- 单次执行时间要短：同队列模块共享线程，某模块超时会拖慢整队列。

### (4)不用WQ的情况
- 若算法必须长期阻塞等待 IO，或单次计算时间不可控且很长，考虑独立任务/独立队列隔离。

---

## 8.3 WQ最小模板组成

建议模块目录：`src/modules/[模块名]/`

| 文件 | 职责 |
|---|---|
| `formation_ctrl.hpp` | 模块类定义（`ModuleBase + ModuleParams + ScheduledWorkItem`） |
| `formation_ctrl.cpp` | 初始化、调度、主循环、参数更新、话题收发 |
| `formation_ctrl_params.c` | `PARAM_DEFINE_*` 参数元数据与默认值（便于 QGC 显示与调参） |
| `CMakeLists.txt` | `px4_add_module()` 构建声明 |
| `Kconfig` | 模块开关与 userspace 条件开关（后者可选） |

---

## 8.4 模板骨架（formation_ctrl为功能名称，可替换）

### (1)CMakeLists.txt
```cmake
px4_add_module(
	MODULE modules__formation_ctrl
	MAIN formation_ctrl
	SRCS
		formation_ctrl.cpp
		formation_ctrl_params.c
	DEPENDS
		px4_work_queue
)
```

### (2)Kconfig
```kconfig
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

endif
```

### (3)formation_ctrl_params.c（参数模板）
```c
#include <px4_platform_common/param.h>

/**
 * Formation control enable
 *
 * @boolean
 * @group Formation
 */
PARAM_DEFINE_INT32(FC_FORM_EN, 0);

/**
 * Formation spacing (m)
 *
 * @unit m
 * @min 0.5
 * @max 50.0
 * @decimal 2
 * @group Formation
 */
PARAM_DEFINE_FLOAT(FC_FORM_D, 5.0f);

/**
 * Position proportional gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Formation
 */
PARAM_DEFINE_FLOAT(FC_FORM_KP, 1.0f);
```

### (4)formation_ctrl.hpp（最小头文件）
```cpp
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>

using namespace time_literals;

class FormationCtrl final : public ModuleBase<FormationCtrl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FormationCtrl();
	~FormationCtrl() override;

	static int task_spawn(int argc, char *argv[]);
	static FormationCtrl *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	bool init();
	void Run() override;
	void parameters_update(bool force = false);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionCallbackWorkItem _lpos_trigger{this, ORB_ID(vehicle_local_position)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FC_FORM_EN>) _param_fc_form_en,
		(ParamFloat<px4::params::FC_FORM_D>) _param_fc_form_d,
		(ParamFloat<px4::params::FC_FORM_KP>) _param_fc_form_kp
	)
};
```

### (5)formation_ctrl.cpp（最小主循环）
```cpp
#include "formation_ctrl.hpp"

FormationCtrl::FormationCtrl()
	: ModuleParams(nullptr)
	, ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

FormationCtrl::~FormationCtrl()
{
	_lpos_trigger.unregisterCallback();
}

bool FormationCtrl::init()
{
	if (!_lpos_trigger.registerCallback()) {
		ScheduleOnInterval(20_ms); // fallback: 50Hz
	}

	return true;
}

void FormationCtrl::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s p{};
		_parameter_update_sub.copy(&p);
		updateParams();
	}
}

void FormationCtrl::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	parameters_update(false);

	vehicle_local_position_s lpos{};
	if (_vehicle_local_position_sub.update(&lpos) && _param_fc_form_en.get() > 0) {
		offboard_control_mode_s ocm{};
		ocm.timestamp = hrt_absolute_time();
		ocm.position = true;
		_offboard_control_mode_pub.publish(ocm);

		trajectory_setpoint_s sp{};
		sp.timestamp = ocm.timestamp;
		sp.x = lpos.x;
		sp.y = lpos.y;
		sp.z = lpos.z;
		sp.yaw = NAN;
		_trajectory_setpoint_pub.publish(sp);
	}
}

int FormationCtrl::task_spawn(int argc, char *argv[])
{
	_task_id = task_id_is_work_queue;
	FormationCtrl *instance = new FormationCtrl();
	_object.store(instance);

	if (instance) {
		_task_id = task_id_is_work_queue;
		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}
```

---

## 8.5 编队控制下需要的话题

### (1)输入（本机状态）
- `vehicle_local_position`
- `vehicle_attitude`（按需）
- `vehicle_status`
- `vehicle_command`（按需）

### (2)输出（控制接口）
- `offboard_control_mode`
- `trajectory_setpoint`

### (3)频率
- 控制主输出（`offboard_control_mode` + `trajectory_setpoint`）：20~50Hz。
- 编队成员状态输入（若通过外部链路接入）：10~20Hz，并做时间戳对齐与轻量滤波。

### (4)降级逻辑
- 编队成员状态数据超时：从“编队跟随”退化为“保持当前位置”设定值。
- 连续超时：退出编队控制使能，等待外部重新激活。

---

## 8.6 Offboard模式下的安全约束
- Offboard生命信号需持续有效（建议持续高于 2Hz，不低于最小要求）。
- 必须配置并理解以下参数：
  - `COM_OF_LOSS_T`（Offboard丢失超时）
  - `COM_OBL_RC_ACT`（丢失后动作）
- 进入Offboard策略：正式接管前，先连续发布一段稳定设定值，避免切入瞬间姿态/位置跳变。
- 退出Offboard策略：退出前把设定值平滑过渡到当前位姿或安全位姿，避免控制突变。

---

## 8.7 启动方式

### 方式A：内置脚本启动
- 在对应机型启动脚本（如 `rc.mc_apps`）增加：
```sh
if param greater -s FC_FORM_EN 0
then
	formation_ctrl start
fi
```
当参数 FC_FORM_EN > 0 时执行 formation_ctrl start

### 方式B：SD卡extras启动
- 在 `/fs/microsd/etc/extras.txt` 添加：
```sh
formation_ctrl start
```
- 优点：不需要重新编译固件，迭代快，适合开发阶段

---

## 8.8 功能验证（SITL/HIL/实机前）

### (1)是否能正常启动
- 模块能正常执行 `start/status/stop`。
- 参数能在地面站中看到并生效，比如 FC_FORM_EN、FC_FORM_D、FC_FORM_KP。
- 控制使能后，输出的设定值应连续，不应出现突跳。

### (2)实时性与负载
- 检查 `work_queue status`：确认队列频率与间隔稳定。
- 检查 uORB 频率：关键输入/输出 topic 无异常掉频。
- 检查 CPU/RAM 与日志中是否出现超时/阻塞迹象。

### (3)安全行为
- 人为断开编队成员状态输入时，系统应触发降级。
- 人为停止 Offboard 输入：应按 `COM_OF_LOSS_T/COM_OBL_RC_ACT` 进入预期 failsafe。
