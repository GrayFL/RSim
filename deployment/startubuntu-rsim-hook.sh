# Insert after the shebang in Android ubuntu-in-termux/startubuntu.sh.
# BootCompleteReceiver -> TestService explicitly executes startubuntu.sh.
# The dedicated entry below does not call startubuntu.sh, so it cannot recurse.
if [ -f /data/data/com.robint/files/home/rsim/android-rsim-service.sh ]; then
    nohup /system/bin/sh /data/data/com.robint/files/home/rsim/android-rsim-service.sh start \
        >>/data/data/com.robint/files/home/rsim/boot.log 2>&1 </dev/null &
fi
