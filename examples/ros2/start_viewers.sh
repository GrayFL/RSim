#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_group=1
LAUNCH_WINDOWS=(rqt rviz)
launch_help() { echo 'Local ROS viewers only: start [rqt rviz]. No service or hardware startup.'; }
launch_command() {
    case $1 in
        rqt) CMD=(rqt "${RQT_ARGS[@]}") ;;
        rviz) CMD=(rviz2 "${RVIZ_ARGS[@]}") ;;
    esac
}
launch_main viewers "$@"
