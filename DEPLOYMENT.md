# leap-ros 本机部署

工作空间：`/home/ros-car/leap-ros`，源码：`src/`。环境为 Ubuntu 22.04 x86_64 / ROS 2 Humble。

源码来自 https://github.com/czu963889306-dev/chuyun_ws ，master 提交 `8043c442ee41c3beb62572502ae071a0ed75b238`。Git 直连失败，本次使用 GitHub 固定提交源码归档部署，`src/` 不含 Git 历史，不能直接 `git pull`。版本及归档校验值见 `source-version.json`。上游源码未修改。

## 加载与编译

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/leap-ros/install/setup.bash
```

重新编译：

```bash
cd ~/leap-ros
./build.sh
```

脚本使用已有的 `vendor/` 源码构建 Micro-XRCE-DDS-Agent v2.4.2，再编译 ROS 工作空间，限制并行度以控制内存占用。`colcon.meta` 仅对 `micro_ros_agent` 关闭联网 superbuild，指定本地依赖及库搜索路径；`vendor/COLCON_IGNORE` 避免 colcon 重复识别依赖源码。无需 sudo。此构建配置按本次要求使用固定路径 `/home/ros-car/leap-ros`。

## 启动

无需硬件的视觉演示：

```bash
ros2 launch chuyun_opencv opencv_demo.launch.py
# 另一个已加载环境的终端查看结果
ros2 topic echo /opencv/detections --once
```

机器人模型：

```bash
ros2 launch chuyun_description display.launch.py
```

连接机器人后，在分别加载环境的终端执行：

```bash
# 终端 1：UDP4 / 8888，需与 MCU 配置一致
ros2 launch micro_ros_agent micro_ros_agent_launch.py

# 终端 2：机器人描述、里程计/IMU 适配与 EKF
ros2 launch chuyun_bringup chuyun_bringup.launch.py

# 终端 3：轻量导航；复用 bringup 已发布的雷达 TF
ros2 launch chuyun_navigation2 lite_navigation.launch.py publish_laser_tf:=false
```

实机要求 MCU 发布 `/odom`、`/imu`，雷达发布 `/scan`，TF 和 ROS_DOMAIN_ID 与设备配置一致。仓库不含独立雷达驱动；bringup 不会启动 micro-ROS Agent。默认导航地图是仓库的 `room.yaml`，可通过 `map:=/绝对路径/地图.yaml` 替换。

HTTP 相机默认地址是 `http://192.168.31.185:81/`，按实际设备覆盖：

```bash
ros2 launch chuyun_camera http_video_publisher.launch.py url:=http://相机IP:81/
```

若需给 OpenCV 提供原始图像，再加 `publish_raw:=true`。Astra 驱动已编译；USB 权限及实际相机采集仍需连接设备后验证。

Wi-Fi 配置节点的 launch 默认网卡为 `wlan0`，本机实际网卡为 `wlp0s20f3`；使用时可运行 `ros2 run chuyun_wifi_config wifi_config_applier --ros-args -p interface:=wlp0s20f3`。部署过程未启动该节点或设置开机服务。

## 验证记录

- 14 个 ROS 包均已构建并在工作空间环境中可定位。
- 11 个已安装 ELF 可执行文件的动态库依赖均可解析。
- Python 相机与 Wi-Fi 节点模块可导入。
- 11 个主要 launch 文件通过 `--show-args` 解析，包括底盘、模型、相机、视觉、micro-ROS、导航和 Cartographer。
- `ctest --test-dir build/chuyun_navigation2 -R '^core_smoke_test$' --output-on-failure` 通过，其中包含 9 项地图、路径规划、扫描匹配、避障和控制测试。
- 在独立 ROS_DOMAIN_ID=215 下启动合成图像 OpenCV 节点，订阅到检测结果：红 1、绿 1、蓝 2，模板匹配分数 1.0，随后已结束测试进程。当前执行沙箱限制 UDP socket，测试通过本机共享内存通信完成；实机网络通信尚未验证。
- 未连接机器人进行运动、雷达或相机采集验证。实际部署到硬件时仍需按设备设置网络、传感器和地图参数。
