import ctypes
from ctypes import wintypes as w
from .files import open_shared


class Overlapped(ctypes.Structure):
    _fields_ = [('internal', ctypes.c_size_t), ('internal_high', ctypes.c_size_t),
                ('offset', w.DWORD), ('offset_high', w.DWORD), ('event', w.HANDLE)]


def open_lock(path):
    # Atomic create-if-absent without truncating a concurrent owner's file.
    from pathlib import Path
    Path(path).touch(exist_ok=True)
    return open_shared(path, write=True)


def try_lock(file):
    import msvcrt
    from .identity import bind
    function = bind(ctypes.WinDLL('kernel32', use_last_error=True), 'LockFileEx', w.BOOL,
                    [w.HANDLE, w.DWORD, w.DWORD, w.DWORD, w.DWORD, ctypes.POINTER(Overlapped)])
    state = Overlapped()
    if not function(msvcrt.get_osfhandle(file.fileno()), 3, 0, 1, 0, ctypes.byref(state)):
        error = ctypes.get_last_error()
        if error == 33:  # ERROR_LOCK_VIOLATION only; access denied is not contention.
            raise BlockingIOError('lock is owned')
        raise ctypes.WinError(error)
