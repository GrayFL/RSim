"""NPY layout and NumPy mappings; all OS operations belong to platforms."""
import os
from pathlib import Path
import ast
import struct
import numpy as np

from rsim.platforms import get_platform
from rsim.platforms.files import open_shared, unlink, remove_tree, hardlink
from rsim.platforms.paths import roots


def storage_root():
    return roots()['storage_root']


def backing_identity(array):
    """Opaque local backing identity, including after its path was unlinked."""
    current = array
    while current is not None:
        identity = getattr(current, '_rsim_backing_identity', None)
        if identity is not None:
            return identity
        if isinstance(current, np.memmap) and current.filename:
            with open_shared(current.filename) as file:
                return get_platform().files.identity(file)
        current = getattr(current, 'base', None)
    raise ValueError('array has no shared backing')


def open_array(path, *, shape=None, dtype=None):
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
        result._rsim_backing_identity = get_platform().files.identity(file)
        return result
