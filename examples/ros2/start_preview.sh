#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() { echo 'Local bounded point-cloud preview only. Subscribes existing hardware topics.'; }
launch_command() {
    CMD=(python -m rsim.apps.pointcloud_preview
        --input "${RSIM_PREVIEW_INPUT:-/iv_points}"
        --output "${RSIM_PREVIEW_OUTPUT:-/rsim/preview/points}"
        --input-domain "${RSIM_PREVIEW_INPUT_DOMAIN:-0}"
        --output-domain "${RSIM_PREVIEW_OUTPUT_DOMAIN:-43}"
        --hz "${RSIM_PREVIEW_HZ:-3}" --max-points "${RSIM_PREVIEW_POINTS:-1200}"
        --max-bytes "${RSIM_PREVIEW_BYTES:-60000}" "$@")
}
launch_main preview "$@"
