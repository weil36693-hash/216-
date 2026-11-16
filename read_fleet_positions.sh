#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${VRPN_SETUP:-$HOME/vrpn_ws/install/setup.bash}" ]]; then
  source "${VRPN_SETUP:-$HOME/vrpn_ws/install/setup.bash}"
fi
export ROS_DOMAIN_ID="${MOCAP_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
exec python3 "$root/scripts/fleet_positions.py" "$@"
