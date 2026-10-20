"""Portable process inspection for lifecycle tests; never signal PID 0 on Windows."""
import psutil


def descendants(parent):
    try:
        return {process.pid for process in psutil.Process(parent).children(recursive=True)}
    except psutil.NoSuchProcess:
        return set()


def alive(pid):
    try:
        process = psutil.Process(pid)
        return process.is_running() and process.status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


def kill(pid):
    psutil.Process(pid).kill()


def suspend(pid):
    psutil.Process(pid).suspend()


def command(pid):
    try:
        return psutil.Process(pid).cmdline()
    except psutil.NoSuchProcess:
        return []
