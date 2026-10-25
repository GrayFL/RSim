#!/bin/bash
# Run inside a fresh PRoot instance (see deployment/android-rsim-service.sh).
set -eu
root=${RSIM_ROOT:-/root/Projects/RSim}
python=${RSIM_PYTHON:-/root/LAB/envs/rsim/bin/python}
config=${RSIM_CONFIG:-$root/assets/robot-hih.json}
data=${RSIM_DATA:-$root/assets/navigation}
run=${RSIM_RUN_DIR:-/dev/shm/rsim-service}
socket=${RSIM_TMUX_SOCKET:-rsim-navigation}
session=rsim
cd "$root"
mkdir -p "$data" "$run"
export RSIM_RUN_DIR="$run"
start() {
    if tmux -L "$socket" has-session -t "$session" 2>/dev/null; then return; fi
    # Separate tmux socket prevents inheriting an old server's /dev/shm binding.
    printf -v command '%q ' "$python" -m rsim.apps.navigation.lifecycle --config "$config" --data-dir "$data"
    tmux -L "$socket" new-session -d -s "$session" -c "$root" "exec $command" || \
        tmux -L "$socket" has-session -t "$session"
}
stop() {
    if ! tmux -L "$socket" has-session -t "$session" 2>/dev/null; then return; fi
    touch "$run/stop.request"
    for i in $(seq 1 40); do
        tmux -L "$socket" has-session -t "$session" 2>/dev/null || return 0
        sleep 1
    done
    printf '%s\n' 'RSim shutdown timed out; inspect the owned session.' >&2
    return 1
}
case "${1:-start}" in
    start) start ;;
    stop) stop ;;
    restart) stop; start ;;
    status) tmux -L "$socket" list-panes -t "$session" -F '#{session_name} #{pane_pid} #{pane_current_command}'; cat "$run/lifecycle.json" ;;
    attach) exec tmux -L "$socket" attach -t "$session" ;;
    run)
        trap 'stop; exit 0' TERM INT
        start
        while tmux -L "$socket" has-session -t "$session" 2>/dev/null; do sleep 2 & wait $! || true; done
        exit 1
        ;;
    *) printf '%s\n' 'Usage: rsim-service.sh start|stop|restart|status|attach|run' >&2; exit 2 ;;
esac
