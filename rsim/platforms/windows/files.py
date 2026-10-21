"""Delete-sharing NTFS handles, file IDs and POSIX deletion."""
import os
from pathlib import Path


def filesystem_info(path):
    import ctypes
    from ctypes import wintypes as w
    from .identity import bind
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    volume = ctypes.create_unicode_buffer(32768)
    get_path = bind(kernel, 'GetVolumePathNameW', w.BOOL, [w.LPCWSTR, w.LPWSTR, w.DWORD])
    if not get_path(str(Path(path).resolve()), volume, len(volume)):
        raise ctypes.WinError(ctypes.get_last_error())
    filesystem = ctypes.create_unicode_buffer(256)
    serial, maximum, flags = w.DWORD(), w.DWORD(), w.DWORD()
    get_info = bind(kernel, 'GetVolumeInformationW', w.BOOL,
                    [w.LPCWSTR, w.LPWSTR, w.DWORD, ctypes.POINTER(w.DWORD), ctypes.POINTER(w.DWORD),
                     ctypes.POINTER(w.DWORD), w.LPWSTR, w.DWORD])
    if not get_info(volume.value, None, 0, ctypes.byref(serial), ctypes.byref(maximum),
                    ctypes.byref(flags), filesystem, len(filesystem)):
        raise ctypes.WinError(ctypes.get_last_error())
    get_type = bind(kernel, 'GetDriveTypeW', w.UINT, [w.LPCWSTR])
    kind = get_type(volume.value)
    return dict(filesystem=filesystem.value, mount=volume.value, device=str(serial.value),
                drive_type=kind, remote=kind == 4)


def open_shared(path, *, write=False, create=False, delete=False):
    """Open with FILE_SHARE_DELETE so retained mappings can survive unlink."""
    import ctypes
    from ctypes import wintypes as w
    import msvcrt
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    create_file = kernel.CreateFileW
    create_file.argtypes = [w.LPCWSTR, w.DWORD, w.DWORD, w.LPVOID, w.DWORD, w.DWORD, w.HANDLE]
    create_file.restype = w.HANDLE
    access = 0x80000000 | (0x40000000 if write else 0) | (0x10000 if delete else 0)
    handle = create_file(str(Path(path).resolve()), access, 7, None, 2 if create else 3, 0x80, None)
    if handle == w.HANDLE(-1).value:
        error = ctypes.get_last_error()
        if error in (2, 3):
            raise FileNotFoundError(str(path))
        raise ctypes.WinError(error)
    fd = msvcrt.open_osfhandle(handle, os.O_BINARY | (os.O_RDWR if write else os.O_RDONLY))
    return os.fdopen(fd, 'r+b' if write else 'rb')


def unlink(path, *, missing_ok=True):
    if Path(path).is_symlink():
        return Path(path).unlink(missing_ok=missing_ok)
    import ctypes
    from ctypes import wintypes as w
    import msvcrt
    try:
        with open_shared(path, delete=True) as file:
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            function = kernel.SetFileInformationByHandle
            function.argtypes = [w.HANDLE, ctypes.c_int, w.LPVOID, w.DWORD]
            function.restype = w.BOOL
            flags = w.DWORD(0x1 | 0x2 | 0x10)  # DELETE | POSIX_SEMANTICS | IGNORE_READONLY
            if not function(msvcrt.get_osfhandle(file.fileno()), 21, ctypes.byref(flags), ctypes.sizeof(flags)):
                raise ctypes.WinError(ctypes.get_last_error())
    except FileNotFoundError:
        if not missing_ok:
            raise


def remove_tree(directory):
    directory = Path(directory)
    if directory.is_junction():
        directory.rmdir()
        return
    if directory.is_symlink():
        directory.unlink()
        return
    if not directory.exists():
        return
    for path in directory.iterdir():
        if path.is_dir() and not path.is_symlink():
            remove_tree(path)
        else:
            unlink(path)
    directory.rmdir()


def hardlink(source, target):
    os.link(source, target)


def identity(file):
    import ctypes
    from ctypes import wintypes as w
    import msvcrt
    from .identity import bind
    class FileId(ctypes.Structure):
        _fields_ = [('volume', ctypes.c_ulonglong), ('identifier', ctypes.c_ubyte * 16)]
    function = bind(ctypes.WinDLL('kernel32', use_last_error=True),
                    'GetFileInformationByHandleEx', w.BOOL,
                    [w.HANDLE, ctypes.c_int, w.LPVOID, w.DWORD])
    info = FileId()
    if not function(msvcrt.get_osfhandle(file.fileno()), 18, ctypes.byref(info), ctypes.sizeof(info)):
        raise ctypes.WinError(ctypes.get_last_error())
    return str(info.volume), bytes(info.identifier).hex()
