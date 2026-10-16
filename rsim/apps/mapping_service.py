"""Run mapping algorithms over existing ROS publishers, without hardware startup."""
import argparse
import asyncio
import json
import logging
import os
from pathlib import Path
import time
import uuid

from rsim.config import load_mapper, read_config
from rsim.runtime import Runtime

logger = logging.getLogger(__name__)


async def run(args):
    settings = read_config(args.config)['mapping']
    if settings.get('start_drivers') or settings.get('connection'):
        raise ValueError('mapping service only subscribes to existing ROS2 topics; start hardware separately')
    session = Path(args.output)/('session-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    session.mkdir(parents=True)
    mapper = load_mapper(args.config, overrides={'start_drivers': False,
        'database': str((session/'rtabmap.db').resolve())})
    # A service lease does not need to import the complete map on every update.
    # Map clients request their own ports when they connect.
    async with Runtime(mapper.pose, mapper.status):
        try:
            frame = await mapper.pose.get(timeout=60)
            print(json.dumps({'ready': 'mapping', 'session': str(session.resolve())}), flush=True)
            while True:
                frame = await mapper.pose.get(after=frame.sequence, timeout=10)
        except Exception:
            status = mapper.status.frames
            (session/'failure.json').write_text(json.dumps({
                'time_ns': time.time_ns(),
                'last_status': status[-1].data if status else None,
                'provider_log': str(session.resolve()/'provider.log'),
            }, indent=2))
            logger.exception('Mapping failed; diagnostics saved in %s', session.resolve())
            raise


def main():
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s [%(process)d] %(name)s: %(message)s')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', default='assets/mapping')
    parser.add_argument('--blas-threads', type=int, default=1,
                        help='BLAS workers in the mapping subprocess (default: 1)')
    try:
        args = parser.parse_args()
        if args.blas_threads < 1:
            parser.error('--blas-threads must be positive')
        # SharedProvider execs a new interpreter. Set its BLAS limits before
        # NumPy/SciPy load there; a limit in this lightweight parent is not enough.
        for name in ('OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'BLIS_NUM_THREADS'):
            os.environ[name] = str(args.blas_threads)
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
