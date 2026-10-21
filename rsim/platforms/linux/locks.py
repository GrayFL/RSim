import fcntl
import os


def open_lock(path):
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    return os.fdopen(fd, 'a+b')


def try_lock(file):
    fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
