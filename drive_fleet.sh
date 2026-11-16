#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export LEAP_FLEET_CONFIG="$workspace_dir/config/fleet.json"
exec ros2 run leap_fleet drive "$@"
