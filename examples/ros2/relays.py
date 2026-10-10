"""Expose existing ROS2 topics locally; no hardware factories."""
import argparse
import asyncio
import signal

from rsim.config.loader import read_config
from rsim.transport.descriptor import TransportConfig



def providers(args):
    from rsim import drivers
    transport = TransportConfig(backend='cyclonedds', domain_id=args.domain)
    sources = {}
    if args.relay_config:
        config = read_config(args.relay_config)
        if set(config) != {'relays'} or not isinstance(config['relays'], dict):
            raise ValueError('relay YAML requires a relays mapping')
        for name, recipe in config['relays'].items():
            if recipe.get('enabled', True):
                recipe = {k: v for k, v in recipe.items() if k != 'enabled'}
                sources['relay:' + name] = drivers.ROS2Topic(**recipe, transport=transport)
    if not sources:
        raise ValueError('no enabled providers')
    return sources


async def run(args):
    from rsim.drivers import serve
    sources = providers(args)
    print('Starting providers: ' + ', '.join(sources), flush=True)
    print(f'Same-host clients use Cyclone DDS domain {args.domain} and matching history/settings.', flush=True)
    main = asyncio.current_task()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, main.cancel)
    try:
        async with asyncio.TaskGroup() as group:
            for name, provider in sources.items():
                group.create_task(serve(provider), name=name)
    except asyncio.CancelledError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--relay-config', required=True, help='existing ROS2 topics to expose locally')
    parser.add_argument('--domain', type=int, default=0, help='same-host shared descriptor DDS domain')
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
