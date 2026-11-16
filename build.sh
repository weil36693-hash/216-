#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash

cmake -S "$workspace_dir/vendor/src/Micro-XRCE-DDS-Agent-v2.4.2" \
  -B "$workspace_dir/vendor/build/Micro-XRCE-DDS-Agent-v2.4.2" \
  -DCMAKE_INSTALL_PREFIX="$workspace_dir/vendor/install" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAGENT_SUPERBUILD=OFF \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON \
  -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_CED_PROFILE=OFF \
  -DUAGENT_P2P_PROFILE=OFF \
  -DUAGENT_BUILD_EXECUTABLE=OFF \
  -DUAGENT_ISOLATED_INSTALL=OFF
cmake --build "$workspace_dir/vendor/build/Micro-XRCE-DDS-Agent-v2.4.2" --parallel 2
cmake --install "$workspace_dir/vendor/build/Micro-XRCE-DDS-Agent-v2.4.2"

export CMAKE_PREFIX_PATH="$workspace_dir/vendor/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
cd "$workspace_dir"
colcon build --base-paths src --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
