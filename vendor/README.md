# 本地 micro-ROS 构建依赖

## 25 车本地扩展（2026-11-11）

`src/cpp/participant/FleetDomain.hpp` 和 `Participant.cpp` 增加了显式启用的
`LEAP_AGENT_DOMAIN_ID=1..99`：覆盖客户端请求的 DDS 域，覆盖创建与复用匹配路径。
未设置该变量时保留上游行为。此扩展用于 `start_fleet.sh` 的每车独立 Agent，
配合车辆网页中的独立 UDP 端口，使固定域 0 的固件也能连接不同底盘 ROS 域。
普通 Docker Agent 不带此扩展，不能替代此脚本使用的本地 Agent。
本地已重建并安装共享库；主工作空间 `build.sh` 也会重建该修改。
升级/重新解压 vendor 源码会丢失扩展，需要重新应用并验证。

## 上游与构建信息

Micro-XRCE-DDS-Agent v2.4.2 是本仓库 micro_ros_agent 默认 superbuild 指定的依赖。这里直接构建固定版本，使用系统已安装的 ROS 2 Humble DDS 库，避免构建时再次通过 Git 下载。

- 上游：https://github.com/eProsima/Micro-XRCE-DDS-Agent
- 版本：v2.4.2
- 上游 commit：`57d086216d01ec43121845d385894a25987f8a2c`
- 源码归档 SHA-256：`586b7a021cbec2b5a93fafcdab8a6f3a520952c2c8f5253d217a1e49bfed497a`
- 本机构建依赖：Fast DDS 2.6.12、Fast CDR 1.0.29、spdlog 1.9.2、fmt 8.1.1。
- 安装目录：`/home/ros-car/leap-ros/vendor/install`
- 已完成 CMake 配置、并行 2 构建、安装及 `ldd` 动态库依赖检查；未启动节点。

源码下载与构建命令：

```bash
cd /home/ros-car/leap-ros
mkdir -p vendor/downloads vendor/src/Micro-XRCE-DDS-Agent-v2.4.2 vendor/build vendor/install
curl -L --fail --silent --show-error \
  https://codeload.github.com/eProsima/Micro-XRCE-DDS-Agent/tar.gz/refs/tags/v2.4.2 \
  -o vendor/downloads/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz
tar --extract --gzip \
  --file vendor/downloads/Micro-XRCE-DDS-Agent-v2.4.2.tar.gz \
  --directory vendor/src/Micro-XRCE-DDS-Agent-v2.4.2 --strip-components=1
source /opt/ros/humble/setup.bash
cmake -S vendor/src/Micro-XRCE-DDS-Agent-v2.4.2 \
  -B vendor/build/Micro-XRCE-DDS-Agent-v2.4.2 \
  -DCMAKE_INSTALL_PREFIX=/home/ros-car/leap-ros/vendor/install \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_P2P_PROFILE=OFF \
  -DUAGENT_BUILD_EXECUTABLE=OFF \
  -DUAGENT_ISOLATED_INSTALL=OFF
cmake --build vendor/build/Micro-XRCE-DDS-Agent-v2.4.2 --parallel 2
cmake --install vendor/build/Micro-XRCE-DDS-Agent-v2.4.2
```

主工作空间构建 `micro_ros_agent` 时需要设置：

```bash
export CMAKE_PREFIX_PATH="/home/ros-car/leap-ros/vendor/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="/home/ros-car/leap-ros/vendor/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

并向 colcon 传入 CMake 参数 `-DMICROROSAGENT_SUPERBUILD=OFF`。工作空间根目录的 `build.sh` 和 `colcon.meta` 已保存这些设置，并为 `micro_ros_agent` 配置指向本地 vendor 库的 RPATH；使用本次构建产物时只需加载 `install/setup.bash`。独立使用 vendor 库的其他程序仍需上述 `LD_LIBRARY_PATH` 或自行设置 RPATH。
