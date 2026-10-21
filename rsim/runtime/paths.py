"""Runtime namespaces, persistent logs and private instance directories."""
import hashlib
from pathlib import Path

from rsim.platforms import get_platform
from rsim.platforms.paths import roots, private_directory, new_directory
from .options import RuntimeOptions


def registry_directory(options=None):
    paths = roots((options or RuntimeOptions()).paths)
    return private_directory(paths['registry_root'])


def host_id(options=None):
    platform = get_platform()
    namespace = str(roots((options or RuntimeOptions()).paths)['storage_root'].resolve())
    identity = f'{platform.name}:{platform.identity.boot_id()}:{platform.identity.user_id()}:{namespace}'
    return hashlib.sha256(identity.encode()).hexdigest()


def instance_directory(prefix, options):
    from rsim.platforms.probe import probe_storage
    probe_storage(str(roots(options.paths)['storage_root']))
    return new_directory(prefix=prefix, root=roots(options.paths)['storage_root'])


def worker_log(directory, options, explicit=None):
    # Explicit legacy log_path wins over the log root; document this precedence.
    if explicit is not None:
        result = Path(explicit).expanduser().absolute()
        result.parent.mkdir(parents=True, exist_ok=True)
        return result
    root = private_directory(roots(options.paths)['log_root'])
    identifier = hashlib.sha256(str(Path(directory).absolute()).encode()).hexdigest()[:12]
    return root / (Path(directory).name + '-' + identifier + '.log')
