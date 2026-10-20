"""Local shared-file primitives, including Windows delete-sharing mappings."""
import os
from pathlib import Path
import shutil
import tempfile
import ast
import struct

import numpy as np


def storage_root():
    return Path(tempfile.gettempdir()) if os.name == 'nt' else Path('/dev/shm')


def open_shared(path, *, write=False, create=False, delete=False):
    """Open with FILE_SHARE_DELETE so retained mappings can survive unlink."""
    if os.name != 'nt':
        return open(path, 'w+b' if create else ('r+b' if write else 'rb'))
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


def open_array(path, *, shape=None, dtype=None):
    if os.name != 'nt':
        if shape is not None:
            return np.lib.format.open_memmap(path, mode='w+', shape=shape, dtype=dtype)
        return np.load(path, mmap_mode='r', allow_pickle=False)
    creating = shape is not None
    with open_shared(path, write=creating, create=creating) as file:
        if creating:
            dtype = np.dtype(dtype)
            shape = tuple(shape) if not isinstance(shape, int) else (shape,)
            header = dict(descr=np.lib.format.dtype_to_descr(dtype), fortran_order=False, shape=shape)
            try:
                repr(header).encode('latin1')
            except UnicodeEncodeError:
                # NPY v3 differs from v2 only in header encoding. NumPy exposes
                # public v1/v2 header helpers but no public v3 writer/reader.
                encoded = repr(header).encode('utf-8')
                padding = (-(12 + len(encoded) + 1)) % 64
                encoded += b' '*padding + b'\n'
                file.write(np.lib.format.magic(3, 0) + struct.pack('<I', len(encoded)) + encoded)
            else:
                np.lib.format.write_array_header_2_0(file, header)
            file.flush()
            order = 'C'
        else:
            version = np.lib.format.read_magic(file)
            if version == (1, 0):
                shape, fortran, dtype = np.lib.format.read_array_header_1_0(file)
            elif version == (2, 0):
                shape, fortran, dtype = np.lib.format.read_array_header_2_0(file)
            elif version == (3, 0):
                size = struct.unpack('<I', file.read(4))[0]
                if size > 10000:
                    raise ValueError('shared NPY header too large')
                header = ast.literal_eval(file.read(size).decode('utf-8'))
                if not isinstance(header, dict) or set(header) != {'descr', 'fortran_order', 'shape'}:
                    raise ValueError('invalid shared NPY header')
                shape, fortran = header['shape'], header['fortran_order']
                if (not isinstance(shape, tuple) or not all(type(n) is int and n >= 0 for n in shape)
                        or type(fortran) is not bool):
                    raise ValueError('invalid shared NPY layout')
                dtype = np.lib.format.descr_to_dtype(header['descr'])
            else:
                raise ValueError(f'unsupported NPY version: {version}')
            order = 'F' if fortran else 'C'
        if np.dtype(dtype).hasobject:
            raise ValueError('object arrays cannot be shared')
        result = np.memmap(file, dtype=dtype, mode='r+' if creating else 'r',
                           offset=file.tell(), shape=shape, order=order)
        # np.memmap receives a descriptor-backed file rather than a path.
        result.filename = str(Path(path).resolve())
        result._rsim_file_id = os.fstat(file.fileno()).st_ino
        return result


def unlink(path, *, missing_ok=True):
    if os.name != 'nt' or Path(path).is_symlink():
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


def remove_tree(directory, *, ignore_errors=False):
    if os.name != 'nt':
        return shutil.rmtree(directory, ignore_errors=ignore_errors)
    directory = Path(directory)
    if not directory.exists():
        return
    if directory.is_junction():
        directory.rmdir()
        return
    if directory.is_symlink():
        directory.unlink()
        return
    try:
        for path in directory.iterdir():
            if path.is_dir() and not path.is_symlink():
                remove_tree(path, ignore_errors=ignore_errors)
            else:
                unlink(path)
        directory.rmdir()
    except OSError:
        if not ignore_errors:
            raise
