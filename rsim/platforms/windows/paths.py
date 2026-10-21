"""Private NTFS directories and endpoint ACL verification."""
import ctypes
from ctypes import wintypes as w
import os
from pathlib import Path
import tempfile

from .identity import bind, user_id, sid_string


def defaults():
    base = Path(os.environ.get('LOCALAPPDATA', Path.home() / 'AppData/Local')) / 'RSim'
    return dict(registry_root=base / 'registry-v3', storage_root=Path(tempfile.gettempdir()),
                log_root=base / 'logs')


def secure_new(path):
    advapi = ctypes.WinDLL('advapi32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    convert = bind(advapi, 'ConvertStringSecurityDescriptorToSecurityDescriptorW', w.BOOL,
                   [w.LPCWSTR, w.DWORD, ctypes.POINTER(w.LPVOID), w.LPVOID])
    get_dacl = bind(advapi, 'GetSecurityDescriptorDacl', w.BOOL,
                    [w.LPVOID, ctypes.POINTER(w.BOOL), ctypes.POINTER(w.LPVOID), ctypes.POINTER(w.BOOL)])
    get_owner = bind(advapi, 'GetSecurityDescriptorOwner', w.BOOL,
                     [w.LPVOID, ctypes.POINTER(w.LPVOID), ctypes.POINTER(w.BOOL)])
    apply = bind(advapi, 'SetNamedSecurityInfoW', w.DWORD,
                 [w.LPWSTR, ctypes.c_int, w.DWORD, w.LPVOID, w.LPVOID, w.LPVOID, w.LPVOID])
    free = bind(kernel, 'LocalFree', w.LPVOID, [w.LPVOID])
    descriptor = w.LPVOID()
    sid = user_id()
    if not convert(f'O:{sid}D:P(A;OICI;FA;;;{sid})(A;OICI;FA;;;SY)', 1, ctypes.byref(descriptor), None):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        present, defaulted, dacl, owner = w.BOOL(), w.BOOL(), w.LPVOID(), w.LPVOID()
        if not get_dacl(descriptor, ctypes.byref(present), ctypes.byref(dacl), ctypes.byref(defaulted)):
            raise ctypes.WinError(ctypes.get_last_error())
        if not get_owner(descriptor, ctypes.byref(owner), ctypes.byref(defaulted)):
            raise ctypes.WinError(ctypes.get_last_error())
        # Setting a DACL needs WRITE_DAC, which an owner has implicitly. Asking
        # for OWNER_SECURITY_INFORMATION as well unnecessarily needs WRITE_OWNER
        # on volumes whose inherited ACL only grants Modify to the current user.
        error = apply(str(path), 1, 0x80000004, None, None, dacl, None)
        if error:
            raise ctypes.WinError(error)
    finally:
        free(descriptor)


def validate_private(path):
    path = Path(path)
    if path.is_symlink() or path.is_junction():
        raise PermissionError(f'RSim path must not be a link: {path}')
    advapi = ctypes.WinDLL('advapi32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    get = bind(advapi, 'GetNamedSecurityInfoW', w.DWORD,
               [w.LPCWSTR, ctypes.c_int, w.DWORD, ctypes.POINTER(w.LPVOID), w.LPVOID,
                ctypes.POINTER(w.LPVOID), w.LPVOID, ctypes.POINTER(w.LPVOID)])
    get_ace = bind(advapi, 'GetAce', w.BOOL, [w.LPVOID, w.DWORD, ctypes.POINTER(w.LPVOID)])
    free = bind(kernel, 'LocalFree', w.LPVOID, [w.LPVOID])
    owner, acl, descriptor = w.LPVOID(), w.LPVOID(), w.LPVOID()
    error = get(str(path), 1, 5, ctypes.byref(owner), None, ctypes.byref(acl), None, ctypes.byref(descriptor))
    if error:
        raise ctypes.WinError(error)
    try:
        allowed = {user_id(), 'S-1-5-18', 'S-1-5-32-544'}  # user, SYSTEM, Administrators
        if sid_string(owner) not in allowed or not acl:
            raise PermissionError(f'RSim path has an untrusted owner or unrestricted ACL: {path}')
        # Python 3.13+ mkdir(mode=0o700) uses OWNER RIGHTS after checking owner.
        allowed.add('S-1-3-4')
        # ACL header: revision/reserved/size/ACE count/reserved (8 bytes).
        count = ctypes.c_ushort.from_address(acl.value + 4).value
        for index in range(count):
            ace = w.LPVOID()
            if not get_ace(acl, index, ctypes.byref(ace)):
                raise ctypes.WinError(ctypes.get_last_error())
            kind = ctypes.c_ubyte.from_address(ace.value).value
            flags = ctypes.c_ubyte.from_address(ace.value + 1).value
            if flags & 0x8:  # INHERIT_ONLY does not grant access to this object.
                continue
            if kind == 0 and sid_string(ace.value + 8) in allowed:
                continue
            if kind != 1:  # Reject unfamiliar grant ACEs; ordinary deny ACEs are safe.
                raise PermissionError(f'RSim path grants access outside this user: {path}')
    finally:
        free(descriptor)


def private_directory(path):
    path = Path(path)
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            path.mkdir()
        except FileExistsError:
            pass
        else:
            try:
                secure_new(path)
            except BaseException as error:
                try:
                    path.rmdir()
                except OSError as cleanup:
                    error.add_note(f'new directory rollback failed: {path}: {cleanup}')
                raise
    validate_private(path)
    return path
