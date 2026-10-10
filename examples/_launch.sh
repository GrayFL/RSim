#!/usr/bin/env bash
# Local tmux wrapper only. No SSH, dependency startup, or hardware discovery.
set -eo pipefail
RSIM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

launch_activate() {
    if [[ -n ${RSIM_CONDA_ENV:-} ]]; then
        if [[ -n ${RSIM_CONDA_SH:-} ]]; then
            source "$RSIM_CONDA_SH"
        elif ! declare -F conda >/dev/null; then
            local base
            base=$(conda info --base)
            source "$base/etc/profile.d/conda.sh"
        fi
        conda activate "$RSIM_CONDA_ENV"
    fi
    # Activation may set ROS/RMW variables. Local role assignments take priority.
    if [[ -f $RSIM_ENV_FILE ]]; then set -a; source "$RSIM_ENV_FILE"; set +a; fi
    export PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1
}

launch_dds() {
    export RSIM_DDS_LOCAL_IP=${RSIM_DDS_LOCAL_IP:?Set RSIM_DDS_LOCAL_IP}
    export RSIM_DDS_PEER_IP=${RSIM_DDS_PEER_IP:?Set RSIM_DDS_PEER_IP}
    export RSIM_DDS_DOMAIN=${RSIM_DDS_DOMAIN:-42}
    export CYCLONEDDS_URI=${RSIM_CONTROL_DDS_URI:-file://$RSIM_ROOT/examples/control/cyclonedds.remote.xml}
}

launch_owned() {
    [[ $(tmux show-options -v -t "$launch_session" @rsim-root 2>/dev/null) == "$RSIM_ROOT" ]] &&
    [[ $(tmux show-options -v -t "$launch_session" @rsim-role 2>/dev/null) == "$launch_role" ]]
}

launch_main() {
    launch_role=$1; shift
    local action=${1:-start}; [[ $# == 0 ]] || shift
    if [[ $action == --help || $action == help ]]; then
        echo "Usage: bash $launch_entry {start|run|plan|status|attach|stop} [arguments]"
        echo "Local settings: configs/$launch_role.env (override with RSIM_ENV_FILE)."
        launch_help; return
    fi
    export RSIM_ENV_FILE=${RSIM_ENV_FILE:-$RSIM_ROOT/configs/$launch_role.env}
    [[ $RSIM_ENV_FILE == /* ]] || RSIM_ENV_FILE=$PWD/$RSIM_ENV_FILE
    if [[ -f $RSIM_ENV_FILE ]]; then set -a; source "$RSIM_ENV_FILE"; set +a; fi
    launch_session=${RSIM_TMUX_SESSION:-rsim-$launch_role}
    [[ $launch_session =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid tmux session name' >&2; return 1; }
    cd "$RSIM_ROOT"
    local windows=(main) selected=() window target i
    if [[ ${launch_group:-0} == 1 ]]; then
        windows=("${LAUNCH_WINDOWS[@]}")
        if [[ $# != 0 ]]; then windows=("$@"); fi
        for window in "${windows[@]}"; do
            [[ " ${LAUNCH_WINDOWS[*]} " == *" $window "* && $window =~ ^[a-zA-Z0-9_-]+$ ]] || {
                echo "Unknown window: $window" >&2; return 2;
            }
        done
    fi
    case $action in
        attach) launch_owned && exec tmux attach-session -t "=$launch_session"; return 1 ;;
        status|stop)
            launch_owned || { echo "No owned session: $launch_session" >&2; return 1; }
            local failed=0
            for window in "${windows[@]}"; do
                target="=$launch_session:$window"
                if ! tmux list-panes -t "$target" >/dev/null 2>&1; then
                    echo "Not running: $window"; continue
                fi
                if [[ $action == status ]]; then
                    tmux list-panes -t "$target" -F 'window=#{window_name} dead=#{pane_dead} exit=#{pane_dead_status}'
                    tmux capture-pane -p -t "$target" -S -12
                    [[ $(tmux display-message -p -t "$target" '#{pane_dead}') == 0 ]] || failed=1
                    continue
                fi
                if [[ $(tmux display-message -p -t "$target" '#{pane_dead}') == 0 ]]; then
                    tmux send-keys -t "$target" C-c
                fi
                for ((i=0; i<150; i++)); do
                    [[ $(tmux display-message -p -t "$target" '#{pane_dead}') == 0 ]] || break
                    sleep .1
                done
                if [[ $(tmux display-message -p -t "$target" '#{pane_dead}') == 0 ]]; then
                    echo "Still shutting down: $window" >&2; failed=1
                else
                    tmux kill-window -t "$target"; echo "Stopped $window"
                fi
            done
            return "$failed" ;;
        start|run|plan) ;;
        *) echo "Unknown action: $action" >&2; return 2 ;;
    esac
    launch_activate
    if [[ $action == run && ${#windows[@]} != 1 ]]; then
        echo 'run requires exactly one hardware window name' >&2; return 2
    fi
    if [[ $action == start ]] && tmux has-session -t "=$launch_session" 2>/dev/null; then
        launch_owned || { echo "Session belongs to another launcher: $launch_session" >&2; return 1; }
    fi
    local env_args=() name
    for name in ${!RSIM_@} DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR; do
        [[ ! -v $name ]] || env_args+=(-e "$name=${!name}")
    done
    for window in "${windows[@]}"; do
        if [[ ${launch_group:-0} == 1 ]]; then
            selected=("$window"); launch_command "$window"
        else
            selected=("$@"); launch_command "$@"
        fi
        target="=$launch_session:$window"
        if [[ $action == plan ]]; then
            printf '%s: ' "$window"; printf '%q ' "${CMD[@]}"; printf '\n'
        elif [[ $action == run ]]; then
            local log_dir=$RSIM_ROOT/assets/bringup/$launch_role
            mkdir -p "$log_dir/ros"
            export ROS_LOG_DIR=$log_dir/ros
            local log_file=$log_dir/$(date +%Y%m%d-%H%M%S)-$window-$$.log
            echo "Log: $log_file"
            # Ctrl-C targets the whole terminal group. Keep the log reader
            # alive until Python finishes zero/release and exit diagnostics.
            exec > >(tee -i -a "$log_file") 2>&1
            exec "${CMD[@]}"
        elif tmux list-panes -t "$target" >/dev/null 2>&1; then
            echo "Session exists: $launch_session/$window; inspect or stop it first." >&2
            return 1
        elif tmux has-session -t "=$launch_session" 2>/dev/null; then
            tmux new-window -d -t "=$launch_session" -n "$window" -c "$RSIM_ROOT" "${env_args[@]}" \
                bash "$launch_entry" run "${selected[@]}" \; set-option -p -t "$target" remain-on-exit on
        else
            tmux new-session -d -s "$launch_session" -n "$window" -c "$RSIM_ROOT" "${env_args[@]}" \
                bash "$launch_entry" run "${selected[@]}" \; \
                set-option -p -t "$target" remain-on-exit on \; \
                set-option -t "$launch_session" @rsim-root "$RSIM_ROOT" \; \
                set-option -t "$launch_session" @rsim-role "$launch_role"
        fi
    done
    [[ $action != start ]] || echo "Started local windows in $launch_session; use status to inspect readiness."
}
