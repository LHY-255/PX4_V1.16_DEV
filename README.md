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

***

# 二.代码执行流程

  ## 1.rcS脚本详细解析

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
  - 存储卡挂载:脚本首先通过检测块设备 /dev/mmcsd0 来尝试挂载 SD 卡到 /fs/microsd 路径 。如果挂载成功且没有发现 .format 文件，则将 STORAGE_AVAILABLE 变量设置为 yes 。若硬件设备不存在，则会尝试查询 MTD 参数分区作为替代方案
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
  3、MTD备选方案与挂载逻辑结束：如果 /dev/mmcsd0 设备不存在，脚本会尝试通过 mft 查询其他可用的存储分区。
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

  ### (2)参数加载与机架匹配
- 代码首先检查 SD 卡上是否存在一个名为`rc.txt`的完全替代脚本,如果有，脚本会直接执行它，并且跳过后面所有的默认启动逻辑，这通常用于底层调试，或者在文件系统损坏无法启动标准流程时进行救急

```
  if [ -f $FRC ]
then
    .$FRC
```

- 如果没有替代脚本，则加载`rc.filepaths`，获取系统中各种配置文件的标准路径。

```
else

	# Load param file location from kconfig
	. ${R}etc/init.d/rc.filepaths

	# Check if /fs/mtd_params is a valid BSON file
	if ! bsondump docsize /fs/mtd_caldata
	then
		echo "New /fs/mtd_caldata size is:"
		bsondump docsize /fs/mtd_caldata
	fi

	#
	# Load parameters.
	#
	# if the board has a storage for (factory) calibration data
	if mft query -q -k MTD -s MTD_CALDATA -v /fs/mtd_caldata
	then
		param load /fs/mtd_caldata
	fi

	param select $PARAM_FILE
	if ! param import
	then
		echo "ERROR [init] param import failed"
		set STARTUP_TUNE 2 # tune 2 = ERROR_TUNE

		bsondump $PARAM_FILE

		if [ -d "/fs/microsd" ]
		then
			# try to make a backup copy
			cp $PARAM_FILE /fs/microsd/param_import_fail.bson

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

	if [ $STORAGE_AVAILABLE = yes ]
	then
		param select-backup $PARAM_BACKUP_FILE
	fi

	if mft query -q -k MFT -s MFT_ETHERNET -v 1
	then
		netman update -i eth0
	fi

	# To trigger a parameter reset during boot SYS_AUTCONFIG was set to 1 before
	if param greater SYS_AUTOCONFIG 0
	then
		# Reset parameters except airframe, parameter version, RC calibration, sensor calibration, flight modes, total flight time, flight UUID
		param reset_all SYS_AUTOSTART SYS_PARAM_VER RC* CAL_* COM_FLTMODE* LND_FLIGHT* TC_* COM_FLIGHT*
	fi

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

	#
	# Start the tone_alarm driver.
	# Needs to be started after the parameters are loaded (for CBRK_BUZZER).
	#
	tone_alarm start

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

	#
	# Start the socket communication send_event handler.
	#
	send_event start

	#
	# Start the resource load monitor.
	#
	load_mon start

	#
	# Start system state indicator.
	#
	rgbled start -X -q
	rgbled_ncp5623c start -X -q
	rgbled_lp5562 start -X -q
	rgbled_is31fl3195 start -X -q

	#
	# Override parameters from user configuration file.
	#
	if [ -f $FCONFIG ]
	then
		echo "Custom: ${FCONFIG}"
		. $FCONFIG
	fi


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

	# Heater driver for temperature regulated IMUs.
	# The heater needs to start after px4io.
	if param compare -s SENS_EN_THERMAL 1
	then
		heater start
	fi


	#
	# RC update (map raw RC input to calibrate manual control)
	#  start before commander
	#
	rc_update start
	manual_control start

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

	#
	# Configure vehicle type specific parameters.
	# Note: rc.vehicle_setup is the entry point for all vehicle type specific setup.
	. ${R}etc/init.d/rc.vehicle_setup

	# Pre-takeoff continuous magnetometer calibration
	if param compare -s MBE_ENABLE 1
	then
		mag_bias_estimator start
	fi

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

	#
	# Start UART/Serial device drivers.
	# Note: rc.serial is auto-generated from Tools/serial/generate_config.py
	#
	. ${R}etc/init.d/rc.serial

	# Must be started after the serial config is read
	rc_input start $RC_INPUT_ARGS

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

	#
	# Play the startup tune (if not disabled or there is an error)
	#
	param compare CBRK_BUZZER 782090
	if [ "$?" != "0" -o "$STARTUP_TUNE" != "1" ]
	then
		tune_control play -t $STARTUP_TUNE
	fi

	#
	# Start the navigator.
	#
	navigator start

	#
	# Start a thermal calibration if required.
	#
	set RC_THERMAL_CAL ${R}etc/init.d/rc.thermal_cal
	if [ -f ${RC_THERMAL_CAL} ]
	then
		. ${RC_THERMAL_CAL}
	fi
	unset RC_THERMAL_CAL

	#
	# Start gimbal to control mounts such as gimbals, disabled by default.
	#
	if param greater -s MNT_MODE_IN -1
	then
		gimbal start
	fi

	# Blacksheep telemetry
	if param compare -s TEL_BST_EN 1
	then
		bst start -X
	fi

	if param compare -s IMU_GYRO_FFT_EN 1
	then
		gyro_fft start
	fi

	if param compare -s IMU_GYRO_CAL_EN 1
	then
		gyro_calibration start
	fi

	# Check for px4flow sensor
	if param compare -s SENS_EN_PX4FLOW 1
	then
		px4flow start -X &
	fi

	payload_deliverer start

	if param compare -s ICE_EN 1
	then
		internal_combustion_engine_control start
	fi

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

	#
	# Start any custom addons from the sdcard.
	#
	if [ -f $FEXTRAS ]
	then
		echo "Addons script: ${FEXTRAS}"
		. $FEXTRAS
	fi

	#
	# Start the logger.
	#
	set RC_LOGGING ${R}etc/init.d/rc.logging
	if [ -f ${RC_LOGGING} ]
	then
		. ${RC_LOGGING}
	fi
	unset RC_LOGGING

	#
	# Set additional parameters and env variables for selected AUTOSTART.
	#
	if ! param compare SYS_AUTOSTART 0
	then
		. ${R}etc/init.d/rc.autostart.post
	fi


	set BOARD_BOOTLOADER_UPGRADE ${R}etc/init.d/rc.board_bootloader_upgrade
	if [ -f $BOARD_BOOTLOADER_UPGRADE ]
	then
		sh $BOARD_BOOTLOADER_UPGRADE
	fi
	unset BOARD_BOOTLOADER_UPGRADE

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
