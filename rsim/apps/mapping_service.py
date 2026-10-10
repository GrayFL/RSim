"""Run mapping algorithms over existing ROS publishers, without hardware startup."""
import argparse
import asyncio
import json
from pathlib import Path
import time
import uuid

from rsim.config import load_mapper, read_config
from rsim.runtime import Runtime


async def run(args):
    settings = read_config(args.config)['mapping']
    if settings.get('start_drivers') or settings.get('connection'):
        raise ValueError('mapping service only subscribes to existing ROS2 topics; start hardware separately')
    session = Path(args.output)/('session-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    session.mkdir(parents=True)
    mapper = load_mapper(args.config, overrides={'start_drivers': False,
        'database': str((session/'rtabmap.db').resolve())})
    async with Runtime(mapper):
        frame = await mapper.pose.get(timeout=60)
        print(json.dumps({'ready': 'mapping', 'session': str(session.resolve())}), flush=True)
        while True:
            frame = await mapper.pose.get(after=frame.sequence, timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', default='assets/mapping')
    try:
        asyncio.run(run(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
