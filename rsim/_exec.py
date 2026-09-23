"""Internal Linux exec trampoline. Does not fork inside a threaded Python process."""
import ctypes
import os
import signal
import sys


def main():
    parent_pid = int(sys.argv[1])
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_PDEATHSIG)")
    if os.getppid() != parent_pid:
        sys.exit(1)
    os.execv(sys.argv[2], sys.argv[2:])


if __name__ == "__main__":
    main()
