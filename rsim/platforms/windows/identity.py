"""Windows boot UUID and token SID (never a username hash)."""
import ctypes
from ctypes import wintypes as w
from functools import lru_cache
import uuid


def bind(library, name, result, arguments):
    function = getattr(library, name)
    function.restype, function.argtypes = result, arguments
    return function


def sid_string(sid):
    advapi = ctypes.WinDLL('advapi32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    convert = bind(advapi, 'ConvertSidToStringSidW', w.BOOL, [w.LPVOID, ctypes.POINTER(w.LPWSTR)])
    free = bind(kernel, 'LocalFree', w.LPVOID, [w.LPVOID])
    result = w.LPWSTR()
    if not convert(sid, ctypes.byref(result)):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        return result.value
    finally:
        free(result)


@lru_cache(maxsize=1)
def user_id():
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    advapi = ctypes.WinDLL('advapi32', use_last_error=True)
    current = bind(kernel, 'GetCurrentProcess', w.HANDLE, [])
    close = bind(kernel, 'CloseHandle', w.BOOL, [w.HANDLE])
    open_token = bind(advapi, 'OpenProcessToken', w.BOOL, [w.HANDLE, w.DWORD, ctypes.POINTER(w.HANDLE)])
    info = bind(advapi, 'GetTokenInformation', w.BOOL, [w.HANDLE, ctypes.c_int, w.LPVOID, w.DWORD, ctypes.POINTER(w.DWORD)])
    token = w.HANDLE()
    if not open_token(current(), 0x8, ctypes.byref(token)):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        length = w.DWORD()
        info(token, 1, None, 0, ctypes.byref(length))
        buffer = ctypes.create_string_buffer(length.value)
        if not info(token, 1, buffer, length, ctypes.byref(length)):
            raise ctypes.WinError(ctypes.get_last_error())
        return sid_string(ctypes.cast(buffer, ctypes.POINTER(w.LPVOID))[0])
    finally:
        close(token)


@lru_cache(maxsize=1)
def boot_id():
    data = ctypes.create_string_buffer(32)
    query = bind(ctypes.WinDLL('ntdll'), 'NtQuerySystemInformation', ctypes.c_long,
                 [ctypes.c_int, w.LPVOID, w.ULONG, w.LPVOID])
    status = query(90, data, len(data), None)
    if status < 0:
        raise OSError(f'cannot read Windows boot identity: {status & 0xffffffff:#x}')
    return str(uuid.UUID(bytes_le=data.raw[:16]))
