#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_help() {
    echo 'Default: pygame window with zero-only output. Add --live for manual movement.'
    echo 'One-shot CLI: run command status | run command move 0 | run command rotate --deg 0'
    echo 'Other arguments pass through to keyboard_control (e.g. --input terminal).'
}
launch_command() {
    launch_dds
    if [[ ${1:-} == command ]]; then
        shift
        CMD=(python -m rsim.apps.chassis_command --name "${RSIM_CHASSIS_NAME:-chassis}"
            --domain "$RSIM_DDS_DOMAIN" "$@")
    else
        local options=() dry_run=1 arg
        for arg in "$@"; do
            if [[ $arg == --live ]]; then dry_run=0; else options+=("$arg"); fi
        done
        CMD=(python -m rsim.apps.keyboard_control --name "${RSIM_CHASSIS_NAME:-chassis}"
            --domain "$RSIM_DDS_DOMAIN" --input pygame
            --config "${RSIM_KEYBOARD_CONFIG:-examples/control/keyboard.example.yaml}")
        [[ $dry_run == 0 ]] || CMD+=(--dry-run)
        CMD+=("${options[@]}")
    fi
}
launch_main remote "$@"
