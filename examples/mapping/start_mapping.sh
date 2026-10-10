#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Mapping algorithms only; camera/lidar/IMU publishers must already exist.'; }
launch_command() {
    CMD=(python -m rsim.apps.mapping_service --config "${RSIM_MAPPING_CONFIG:-configs/mapping_topics.yaml}"
        --output "$RSIM_ROOT/assets/mapping" "$@")
}
launch_main mapping "$@"
