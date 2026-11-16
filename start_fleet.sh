#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
exec python3 "$root/scripts/fleet_bringup.py" "$@"
