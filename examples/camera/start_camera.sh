#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Local D435 hardware provider only; append native --ros-args as needed.'; }
launch_command() {
    CMD=(python -m rsim.drivers d435 --backend cyclonedds --domain "${RSIM_SHARED_DOMAIN:-0}"
        --serial "${RSIM_CAMERA_SERIAL:-}" --history 8 --stream color
        --color-profile "${RSIM_COLOR_PROFILE:-640x480x15}" --depth-profile "${RSIM_DEPTH_PROFILE:-640x480x15}"
        --log-path "$RSIM_ROOT/assets/bringup/camera/driver.log" "$@")
}
launch_main camera "$@"
