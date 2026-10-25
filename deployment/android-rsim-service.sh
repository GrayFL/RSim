#!/system/bin/sh
set -eu
prefix=/data/data/com.robint/files
exec /system/bin/sh "$prefix/home/ubuntu-in-termux/ubuntu-tmpfs-exec.sh" \
    /bin/bash /root/Projects/RSim/scripts/rsim-service.sh "${1:-run}"
