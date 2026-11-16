# leap_ros_mcu_driver MAVLink 与 micro-ROS 协议说明

软件版本：`v2.2.1-motion-safety`

本文档说明 `leap_ros_mcu_driver` 固件提供的网络与串口通信协议。

## 运行模式

固件同一时间只启用一种通信模式：

| 模式 | 运行值 | 默认 |
| --- | --- | --- |
| micro-ROS | `micro_ros` | 是 |
| micro-ROS UART | `microros_uart` | 否 |
| MAVLink UDP | `mavlink_udp` | 否 |
| MAVLink UART | `uart_mavlink` | 否 |

通信模式保存在 NVS 的运行配置中，可通过网页配置页面修改。默认 micro-ROS agent 地址为 `192.168.31.214:8888`。

## 单位约定

| 数据 | 内部单位 | MAVLink 单位 | ROS 单位 |
| --- | --- | --- | --- |
| 线性位置 | mm | m | m |
| 线速度 | mm/s | m/s | m/s |
| 航向角 / 角速度 | rad / rad/s | rad / rad/s | rad / rad/s |
| IMU 加速度 | g | `SCALED_IMU` 中为 mg，ROS 中为 m/s^2 | m/s^2 |
| IMU 陀螺仪 | deg/s | rad/s 或 mrad/s | rad/s |
| 雷达距离 | mm | `OBSTACLE_DISTANCE` 中为 cm | m |
| 电池电压 | V | mV | V |

## micro-ROS

`micro_ros` 模式使用 Wi-Fi UDP；`microros_uart` 模式使用与 MAVLink UART 相同的 UART0，不依赖 Wi-Fi 连接。UART 参数为 GPIO43 (TX)、GPIO44 (RX)、921600 波特率、8N1、无硬件流控。进入 UART 模式后固件默认等待系统启动 8 秒，再建立 micro-ROS 会话，无需额外握手；串口接收数据按二进制 XRCE-DDS 帧处理，不打印接收内容。

`micro_ros` 传输方式为基于 Wi-Fi 的自定义 UDP；`microros_uart` 传输方式为 UART0 自定义串口传输。

| 项目 | 值 |
| --- | --- |
| 节点名称 | `leap_ros` |
| 本地 UDP 端口 | `CONFIG_MICRO_ROS_LOCAL_PORT`，默认 `8888` |
| Agent 地址 | 运行时 `g_microros_agent_ip:g_microros_agent_port`，默认 `192.168.31.214:8888` |
| 定时器周期 | 20 ms |

### 订阅话题

| 话题 | 类型 | QoS | 映射 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | sensor data | `linear.x/y` 由 m/s 转为目标 `vx/vy`，单位 mm/s；`angular.z` 转为目标 `wz`，单位 rad/s |
| `/host_info` | `std_msgs/msg/String` | sensor data | 目标主机信息：`hostname=<name> ip=<ipv4>` 或 `hostname=<name> hostip=<ipv4>` |

有效 `/cmd_vel` 由 m/s 数值转换为内部 mm/s，写入运动指令队列，来源标记为 micro-ROS，并清除 emergency stop。非有限值或无法用内部 float 表示的速度被拒绝，不刷新租约。ROS 消息字段仍是标准 float64，没有改变消息定义或序列化 ABI。

### 速度租约与状态检查

micro-ROS 速度模式使用 500 ms 车端接收租约，由独立 20 ms 电机控制任务检查。发送端应持续刷新指令，建议 20 Hz；停止时仍应重复发送零速。最后一条有效速度过期后，控制任务清除有效速度及缓存速度并制动。手柄接管期间也继续计算租约，释放手柄只允许恢复尚未过期的 micro-ROS 指令。该租约不改变 MAVLink、位置控制或手柄自身的控制协议。

`/cmd_vel` 沿用 sensor data QoS（best effort）。电脑端 DDS 的可靠发布或 ACK 不能证明 XRCE 链路投递、车端回调执行或电机停车。标准 Twist 没有发送时间戳/序号；租约从车端收到有效消息开始，不能识别已在网络中延迟的旧指令。新增序号仅为车端本地计数，不是发送端序号或停车执行 ACK。

只读 `GET /api/status` 增加：

```json
{
  "firmware": {"version": "2.2.1-motion-safety", "motion_safety_revision": 1},
  "cmd_vel_watchdog": {
    "enabled": true,
    "timeout_ms": 500,
    "has_command": false,
    "last_cmd_age_ms": null,
    "timed_out": false,
    "received_sequence": 0,
    "applied_sequence": 0
  }
}
```

`has_command` 表示控制任务已观察过有效 micro-ROS 速度；`last_cmd_age_ms` 是距该消息车端接收时刻的毫秒数，未收到时为 null。`timed_out` 表示最后一条 micro-ROS 速度租约已过期，即使车辆当前由手柄控制也可能为 true。`received_sequence` 是最近被运动队列交给控制任务的有效接收计数（队列可覆盖中间消息）；`applied_sequence` 是最近传给速度控制器的计数。两者均为 uint32，可回绕，不证明物理停车。安装旧固件时这些对象缺失，必须显示“未报告”。

HTTP `motion.vx/vy/vel_left/vel_right` 为编码器滤波速度（mm/s），`wz` 为 rad/s；ROS `/odom` 线速度为 m/s。这次同时修复 `target_vy/target_wz` 未拷贝的遥测错误。编码器零输入且滤波尾数低于 0.001 rpm 时归零；约 `1e-48 m/s` 的旧值可由 float IIR 非规格化残值精确产生，并非足以证明 float64 序列化错误。

已知重连边界：现有通信循环主要在 executor 返回错误时重建会话。Agent 重启后若等待持续超时且没有明确 executor 错误，旧会话可能不及时重建；此版本只增加独立速度租约，不把 Agent 可 ping 视为会话有效，也不增加未经验证的周期强制重连。

### I²C 手柄控制

固件以 20 Hz 轮询地址 `0x50` 的 I²C 手柄。有效摇杆输入进入独立的手动控制通道，使用与 `/cmd_vel` 相同的速度语义，不会覆盖导航命令：

| 手柄输入 | 控制量 | 最大值 |
| --- | --- | --- |
| 左摇杆 Y | 前后线速度 `linear.x` | `300 mm/s`（等价于 `0.3 m/s`） |
| 右摇杆 X | 转向角速度 `angular.z` | `2.5 rad/s` |

摇杆死区为 12（原始范围 `-127..127`）。摇杆离开死区时手柄临时接管底盘；回到死区、断连或数据帧无效时，仅释放手动接管，不发送零速 `/cmd_vel`，随后仅继续执行仍在有效租约内的 micro-ROS 导航速度；已过期时保持零速。其它来源沿用原协议。

### 时间戳同步

UDP 和 UART 模式均与 micro-ROS Agent 同步时间。首次同步超时为 1000 ms；失败后每隔 1 秒重试，成功后每 30 秒重新同步。运行时同步的超时为 20 ms。

当前会话同步成功后才发布 `/odom`、`/imu`、`/scan`、`/battery_state` 和 `/ultrasonic`。同步失败、同步标志丢失、时间无效或距离上次成功同步超过 90 秒时，这些话题暂停，直到同步恢复；`/wifi_config`、订阅和服务继续运行。重建会话时会重新同步。

`header.stamp` 使用 Agent 主机系统时间对应的发布时刻，精度为毫秒；当前传感器队列不包含采样时间，且此机制不使用 ROS 仿真时间 `/clock`。

可在主机上运行回归测试：

```bash
cmake -S tests -B /tmp/leap-time-sync-tests
cmake --build /tmp/leap-time-sync-tests
ctest --test-dir /tmp/leap-time-sync-tests --output-on-failure
```

### 发布话题

| 话题 | 类型 | 频率 | 坐标系 | 说明 |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | `odom`，子坐标系 `base_link` | 来自运动状态的位置与速度 |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz | `imu_link` | 四元数、陀螺仪、加速度 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | `laser_frame` | 360 个点，角度增量 1 度，范围 0.02-12.0 m |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 10 Hz | `battery` | 电压与电量百分比；锂电池，放电状态 |
| `/ultrasonic` | `sensor_msgs/msg/Range` | 10 Hz | `ultrasonic_link` | HC-SR04 超声波距离，单位 m |
| `/wifi_config` | `std_msgs/msg/String` | 1 Hz | 无 | 已保存的 STA 凭据 JSON：`{"ssid":"<name>","password":"<password>"}` |

雷达在 UART1（RX GPIO41）自动识别 STL26（230400 波特率）与欢创 Camsense（115200 波特率）。每种波特率探测 1.5 秒，收到 3 个有效协议帧后锁定；连续 2 秒无有效帧则清空旧扫描并重新探测。

STL26 不加角度偏置，欢创使用 18.5° 偏置，两种雷达采用相同方向镜像，无扇区屏蔽和额外的 10 cm 近距过滤。整圈按 0–359° 的 360 个方向保存同角度最近的有效距离，避免超过 500 个采样点时丢失后续角度。无有效回波的方向仍为 0。

`/battery_state` 字段：

| 字段 | 值 |
| --- | --- |
| `voltage` | 实测电池电压，单位 V |
| `percentage` | `0.0` 到 `1.0` |
| `power_supply_status` | `DISCHARGING` |
| `power_supply_health` | `GOOD` |
| `power_supply_technology` | `LIPO` |
| `present` | `true` |
| `temperature/current/charge/capacity/design_capacity` | `NaN` |

## MAVLink UDP

传输方式：绑定到 `14550` 端口的 UDP socket。

| 项目 | 值 |
| --- | --- |
| 本地端口 | `14550` |
| 初始目标 | 广播地址 `255.255.255.255:14550` |
| 已连接目标 | 第一个发送 MAVLink 数据的地址 |
| System ID | `1` |
| Component ID | `1` |
| 主循环周期 | 20 ms |
| 车辆类型 | `MAV_TYPE_GROUND_ROVER` |
| Autopilot | `MAV_AUTOPILOT_GENERIC` |

## MAVLink UART

传输方式：`UART0`，波特率 `230400`，8N1。运行日志与 MAVLink UART 共用 UART0 串口。

| 项目 | 值 |
| --- | --- |
| TX 引脚 | GPIO43 |
| RX 引脚 | GPIO44 |
| 日志端口 | 共用 UART0 |
| System ID | `1` |
| Component ID | `1` |
| 主循环周期 | 20 ms |

## MAVLink 消息

### 接收消息

| 消息 | 作用 |
| --- | --- |
| `SET_POSITION_TARGET_LOCAL_NED` | 启用 x/y/yaw 字段时作为位置指令；启用 vx/vy/yaw_rate 字段时作为速度指令 |
| `COMMAND_LONG` | 处理解锁/上锁、里程计复位、舵机、直接轮速、相对运动和 PID 更新指令 |
| `PARAM_REQUEST_READ` | 返回单个 PID 参数 |
| `PARAM_REQUEST_LIST` | 返回全部 PID 参数 |
| `PARAM_SET` | 更新单个 PID 参数并返回新值 |

### 服务

| 服务 | 作用 |
| --- | --- |
| `/set_speed_pid` | 设置速度 PID 的 `kp/ki/kd`，并写入 NVS |
| `/get_speed_pid` | 获取当前速度 PID 参数 |

### COMMAND_LONG 指令

| 指令 | 参数 | 结果 |
| --- | --- | --- |
| `MAV_CMD_COMPONENT_ARM_DISARM` | `param1 < 0.5`：停止；否则清除急停 | `ACCEPTED` |
| `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS` | 无 | 复位里程计 |
| `MAV_CMD_DO_SET_SERVO` | `param2`：舵机角度 | 写入 `q_servo_cmd` |
| `MAV_CMD_DO_SET_ACTUATOR` | `param1`：左轮目标，`param2`：右轮目标 | 轮速模式 |
| `MAV_CMD_USER_2` | `param1`：相对距离，`param2`：相对航向角 | 相对运动模式 |
| `MAV_CMD_USER_4` | `param1`：PID 目标，`param2`：kp，`param3`：ki，`param4`：kd | 更新速度或位置 PID |

自定义指令 ID：

| 名称 | 值 | 含义 |
| --- | --- | --- |
| `MATURO_MAV_CMD_MOVE_RELATIVE` | `MAV_CMD_USER_2` | 相对运动 |
| `MATURO_MAV_CMD_SET_PID` | `MAV_CMD_USER_4` | 设置 PID 参数 |

PID 目标值：

| 值 | 目标 |
| --- | --- |
| `1` | 速度 PID |
| `2` | 位置 PID |

### 参数

MAVLink 参数协议暴露以下参数：

| 参数 | 类型 | 含义 |
| --- | --- | --- |
| `SPD_KP` | `MAV_PARAM_TYPE_REAL32` | 速度 PID kp |
| `SPD_KI` | `MAV_PARAM_TYPE_REAL32` | 速度 PID ki |
| `SPD_KD` | `MAV_PARAM_TYPE_REAL32` | 速度 PID kd |
| `POS_KP` | `MAV_PARAM_TYPE_REAL32` | 位置 PID kp |
| `POS_KI` | `MAV_PARAM_TYPE_REAL32` | 位置 PID ki |
| `POS_KD` | `MAV_PARAM_TYPE_REAL32` | 位置 PID kd |

### 发布消息

有对应源数据时每 20 ms 发布：

| 消息 | 内容 |
| --- | --- |
| `ATTITUDE` | roll/pitch/yaw 与陀螺仪，单位 rad |
| `ATTITUDE_QUATERNION` | 四元数与陀螺仪，单位 rad/s |
| `SCALED_IMU` | 加速度单位 mg，陀螺仪单位 mrad/s，磁力计为 0 |
| `ODOMETRY` | 本地 FLU 位姿与机体 FRD 速度 |

每 100 ms 发布：

| 消息 | 内容 |
| --- | --- |
| `DISTANCE_SENSOR` | 超声波距离，2-400 cm |
| `DEBUG` | index `1`，运动忙状态以 `0.0` 或 `1.0` 表示 |
| `RAW_RPM` | index `0`：左轮 RPM；index `1`：右轮 RPM |

每完成一帧雷达扫描后发布：

| 消息 | 内容 |
| --- | --- |
| `OBSTACLE_DISTANCE` | 360 度雷达扫描拆分为 5 个数据包；每包 72 个 1 度 bin，`angle_offset` 分别为 `0/72/144/216/288` 度 |

每 1 s 发布：

| 消息 | 内容 |
| --- | --- |
| `HEARTBEAT` | 地面车心跳 |
| `ONBOARD_COMPUTER_STATUS` | 堆内存、Flash、Wi-Fi 链路估计、有效时的板载温度 |
| `SYS_STATUS` | 电池电压，单位 mV；剩余电量百分比 |
| `BATTERY_STATUS` | 锂电池电压与剩余电量百分比 |

首次看到 MAVLink 客户端后发布一次：

| 消息 | 内容 |
| --- | --- |
| `COMPONENT_INFORMATION_BASIC` | vendor `Maturo`，model `Driver`，software `v2.2`，hardware `ESP32`，serial 为设备名 |

MAVLink 发布行为遵循当前启用的通信模式和对应传输方式。

### 电池上报

电池状态基于 GPIO3 ADC 采样：

| 字段 | 值 |
| --- | --- |
| 满电电压 | `8.4 V` |
| 空电估计电压 | `6.0 V` |
| 分压比 | `8.4 / 2.6857` |
| 百分比 | 根据 6.0 V 到 8.4 V 线性估算 |

MAVLink 电池字段：

| 消息 | 字段 | 值 |
| --- | --- | --- |
| `SYS_STATUS` | `voltage_battery` | 电池电压，单位 mV；无效时为 `UINT16_MAX` |
| `SYS_STATUS` | `current_battery` | `-1`，表示未知 |
| `SYS_STATUS` | `battery_remaining` | `0..100`，无效时为 `-1` |
| `BATTERY_STATUS` | `battery_function` | `MAV_BATTERY_FUNCTION_ALL` |
| `BATTERY_STATUS` | `type` | `MAV_BATTERY_TYPE_LIPO` |
| `BATTERY_STATUS` | `voltages[0]` | 电池电压，单位 mV |
| `BATTERY_STATUS` | `current_battery/current_consumed/energy_consumed` | `-1`，表示未知 |
| `BATTERY_STATUS` | `charge_state` | 正常；低电量为 <=20%；严重低电量为 <=10% |

## 模式切换说明

只有当前启用的模式会发送通信数据：

| 当前模式 | 行为 |
| --- | --- |
| `micro_ros` | 创建并运行 micro-ROS 实体；MAVLink UDP 和 MAVLink UART 不发布、不接收 |
| `mavlink_udp` | MAVLink UDP 任务接收和发布；micro-ROS 与 MAVLink UART 不工作 |
| `uart_mavlink` | MAVLink UART 任务接收和发布；micro-ROS 与 MAVLink UDP 不工作 |

网页运行配置会将通信模式和 micro-ROS agent 地址保存到 NVS，重启后仍然生效。

## BOOT 键回到配网模式

运行时长按 BOOT 键约 3 秒会清除已保存的 Wi-Fi STA 配置，状态灯进入快闪。松开 BOOT 键后设备会重启，并进入 Wi-Fi 配网模式。

## 2026.11.16-runtime1：运行时实验参数

网页 `http://底盘IP/tuning` 可修改以下参数；首页提供“实验参数”入口。
它们也支持 `GET /api/tuning`、`POST /api/tuning`（JSON 部分更新），
保存成功后立即生效并持久化到 NVS `experiment_cfg/tuning_v1`。
重启会加载原值；缺少记录采用默认值；损坏记录报告错误并保留，不擦除 NVS。

| JSON 参数 | 默认 | 允许范围 | 作用 |
|---|---:|---:|---|
| `telemetry_hz` | 50 | 整数 1–50 Hz | micro-ROS `odom`、`imu`；MAVLink ATTITUDE、ATTITUDE_QUATERNION、SCALED_IMU、ODOMETRY |
| `slow_telemetry_hz` | 10 | 整数 1–10 Hz | micro-ROS `scan`、`battery_state`、`ultrasonic`；MAVLink 距离、轮速、忙状态及新雷达扫描 |
| `gyro_scale_x` | 1.0 | 0.5–2.0 | X 陀螺角速度比例 |
| `gyro_scale_y` | 1.0 | 0.5–2.0 | Y 陀螺角速度比例 |
| `gyro_scale_z` | 1.0 | 0.5–2.0 | Z 陀螺角速度比例 |

```sh
curl http://底盘IP/api/tuning
curl -X POST http://底盘IP/api/tuning -H 'Content-Type: application/json' \
  --data '{"telemetry_hz":20,"slow_telemetry_hz":5,"gyro_scale_z":1.035}'
```

成功返回当前配置、`saved:true`、`restart:false`；不存在或重复字段、非法类型、
非有限数、越界值返回 400，写入失败返回 500 且当前生效参数不变。
GET 返回的 `schema_version` 和 `firmware_version` 是只读字段，不应原样提交。
多个客户端应先读取再按需提交字段；同一 HTTP 服务顺序处理更新。

遥测调度仍在现有 20 ms 服务周期内执行，各周期可能有一个调度步长的抖动；
频率表示希望达到的发布速率，网络拥塞、主机时钟同步失败、传感器产出不足
会降低有效数据率。ROS 雷达发送队列中的最近一圈数据，可能重复同一圈；
MAVLink 仅发送新扫描，`slow_telemetry_hz` 是其发送上限。不能由此改变雷达转速。
1 Hz MAVLink 心跳、系统状态/电量、Wi-Fi 配置信息和 ROS Wi-Fi 配置广播
保持独立，不随上述参数改变。运动控制、指令接收和看门狗，以及 IMU 采样
的 20 ms 周期也保持不变。

陀螺比例在原来的原始计数→度/秒转换并减去开机零偏后应用，
再进入原来的 0.2 dps 死区和姿态积分；不改变轴方向和 ROS 的 rad/s 单位。
修改比例影响之后的角速度和姿态更新，不清零已经累计的 yaw。
开机静止零偏校准仍每次进行，比例系数单独保存在 NVS。

### Wi-Fi 配置与省电

原有网页和表单 `POST /api/provision` 保持兼容。新增 JSON 接口：

```sh
curl http://底盘IP/api/wifi
curl -X POST http://底盘IP/api/wifi -H 'Content-Type: application/json' \
  --data '{"ssid":"实验网络","password":"替换为实际密码"}'
```

`ssid` 是要连接的路由器名称，1–32 UTF-8 字节；`password` 要同时提供，
开放网络为空字符串，其他网络为 8–63 字节。GET 仅返回 SSID、是否设置密码
和实际省电模式，不返回密码。POST 保存到现有 `wifi_cfg`，成功返回
`{"saved":true,"restart":true,"delay_ms":1200}`，随后重启并连接新网络。
因此无需重新刷固件，但 Wi-Fi 配置修改会中断当前会话，应在小车停止后操作。
`Maturo_<MAC>` 是固定设备身份，不是此接口要修改的路由器 SSID。

STA、APSTA 和 ESP-NOW 初始化均明确调用 `esp_wifi_set_ps(WIFI_PS_NONE)`。
`GET /api/status` 的 `wifi.power_save` 与 `/api/wifi` 的 `power_save`
报告驱动实际值，0 表示关闭省电；读取失败为 null，不假定成功。

### 固件身份

`GET /api/status` 保留原有设备名、camera 信息及 `firmware.version`，并返回：

```json
{"firmware":{"version":"2026.11.16-runtime1","motion_safety_revision":1,
 "idf_version":"v5.5.3","project_name":"leap_ros_mcu_driver",
 "build":"编译日期 编译时间","app_sha256":"64位十六进制"}}
```

`app_sha256` 来自 ESP-IDF 应用描述中的 ELF SHA256，**不是应用 bin 文件的 SHA256**。
版本也写入启动日志和 MAVLink COMPONENT_INFORMATION_BASIC 的 software_version。
会话日志采集端应同时记录设备身份、固件版本和观测时间，读取失败应明确记为未知。
