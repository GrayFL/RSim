"""Persistent output ownership, separate from generated session parameters."""
from pathlib import Path

from rsim.core import Component
from rsim.runtime.locks import acquire_device


class MappingDatabase(Component):
    """Own one new mapping session without deleting any user-owned asset.

    Each graph has its own owner object, even when paths match, so a second
    writer is rejected rather than silently deduplicated into the same Runtime.
    """
    def __init__(self, path, *, policy='new'):
        if policy != 'new':
            raise ValueError('only a new database session is supported')
        super().__init__()
        self.path = Path(path).expanduser().resolve()
        self.archive = self.path.with_suffix('.laser')
        self.policy = policy
        self.lease = None
        self.archive_lease = None

    async def open(self):
        try:
            self.lease = acquire_device('mapping-asset:' + str(self.path))
            self.archive_lease = acquire_device('mapping-asset:' + str(self.archive))
            if self.path.exists() and self.path.stat().st_size:
                raise ValueError('start a new database for a new laser map session; native-map relocalization is not implicit')
            if self.archive.exists() and (not self.archive.is_dir() or any(self.archive.iterdir())):
                raise ValueError('start a new archive for a new laser map session; existing assets are preserved')
            self.path.parent.mkdir(parents=True, exist_ok=True)
        except BaseException:
            await self.close()
            raise

    async def close(self):
        if self.archive_lease is not None:
            self.archive_lease.close()
            self.archive_lease = None
        if self.lease is not None:
            self.lease.close()
            self.lease = None
