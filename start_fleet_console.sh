#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export LEAP_FLEET_CONFIG="$workspace_dir/config/fleet.json"
export LEAP_FLEET_WORKSPACE="$workspace_dir"
exec python3 -m leap_fleet.web_console "$@"
