#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Local external IMU hardware provider only.'; }
launch_command() {
    CMD=(python -m rsim.drivers imu --backend cyclonedds --domain "${RSIM_SHARED_DOMAIN:-0}"
        --port "${RSIM_IMU_PORT:?Set RSIM_IMU_PORT}" --imu-mode ros2 --baudrate 460800 --history 128
        --frame-id "${RSIM_IMU_FRAME:-hipnuc_imu}"
        --log-path "$RSIM_ROOT/assets/bringup/imu/driver.log" "$@")
}
launch_main imu "$@"
