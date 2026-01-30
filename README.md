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
CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF=n     #关闭VTOL的相关模块
CONFIG_MODULES_VTOL_ATT_CONTROL=n       #关闭VTOL的相关模块

无人车、无人潜艇等模块是默认关闭的，可以不用设置。如果要设置，可以在
/PX4-Autopilot/boards/px4/fmu-v6x/default.px4board文件中显式开启，也可以在PX4/PX4-Autopilot/src/modules中对应模块的Kconfig文件中开启或者关闭。
```

**注意，在PX4代码中不要使用中文注释，会报错！**


# 二.代码执行流程

  ## 1.rcS文件详细解析

飞控硬件上电并且完成`Bootloader`引导后，由实时操作系统`NuttX`调用第一个启动脚本：`PX4-Autopilot/ROMFS/px4fmu_common/init.d/rcS`，该脚本主要完成如下任务：

  ### (1)环境与存储初始化
  - 基础设置：脚本首先初始化全局变量。
  ```
1、脚本环境与调试控制
#!/bin/sh  #指定脚本解释器为系统的shell程序
set +e  #设置“忽略错误继续执行”模式。这意味着即使某行命令运行失败，脚本也不会立即退出，确保系统能尽可能多的初始化硬件
#set -x  #此行被注释掉，若启用，系统会打印出脚本执行的每一条指令，通常用于底层调试
#------------------------------------------------------------------------------
2、基础路径变量设置
set R /  #定义根目录变量${R}为 /
set FCONFIG /fs/microsd/etc/config.txt  #定义用户自定义配置文件的路径，位于 SD 卡中
set FEXTRAS /fs/microsd/etc/extras.txt  #定义额外启动脚本的路径，用于加载用户自定义模块
set FRC /fs/microsd/etc/rc.txt  #定义备用启动脚本路径，若此文件存在，通常会改变默认启动流程
set IOFW "/etc/extras/px4_io-v2_default.bin"  #指定PX4IO协同处理器的固件镜像路径，用于后续的固件版本校验或更新
#------------------------------------------------------------------------------
3、系统状态与日志参数初始化
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
  - 存储卡挂载:尝试挂载 microSD 卡（/fs/microsd），并检查是否存在格式化请求（.format文件）
  - 挂载SD卡部分技术栈为FAT32
  ```
1、尝试挂载SD卡
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
  #------------------------------------------------------------------------------
  2、执行格式化逻辑：如果初次挂载失败或检测到格式化标记，脚本会执行以下格式化并重新挂载的操作
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
  #------------------------------------------------------------------------------
  3、MTD备选方案与挂载逻辑结束
  ```

  ### (2)参数加载与机架匹配
