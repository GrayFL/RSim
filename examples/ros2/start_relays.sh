#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() {
    echo 'Expose existing ROS2 topics as same-host rsim.devices.ROS2Topic providers.'
    echo 'Arguments pass to examples.ros2.relays; no hardware is started.'
}
launch_command() {
    CMD=(python -m examples.ros2.relays
        --relay-config "${RSIM_RELAY_CONFIG:-configs/relays.yaml}"
        --domain "${RSIM_SHARED_DOMAIN:-0}" "$@")
}
launch_main relays "$@"
