import os
from pathlib import Path


def user_id():
    return str(os.getuid())


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
