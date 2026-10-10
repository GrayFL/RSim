#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Local RobinW hardware provider only; append native --ros-args as needed.'; }
launch_command() {
    CMD=(python -m rsim.drivers robin --backend cyclonedds --domain "${RSIM_SHARED_DOMAIN:-0}"
        --ip "${RSIM_LIDAR_IP:?Set RSIM_LIDAR_IP}" --history 8
        --log-path "$RSIM_ROOT/assets/bringup/lidar/driver.log" "$@")
}
launch_main lidar "$@"
