#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Topic-only controller service on this host. Hardware must be started separately.'; }
launch_command() {
    launch_dds
    CMD=(python -m rsim.apps.chassis_service --config "${RSIM_CHASSIS_CONFIG:-configs/chassis_topics.yaml}"
        --name "${RSIM_CHASSIS_NAME:-chassis}" --domain "$RSIM_DDS_DOMAIN" "$@")
}
launch_main chassis "$@"
