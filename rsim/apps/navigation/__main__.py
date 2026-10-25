"""Run an independent RSim ROS1 navigation service (zero-only by default)."""
import argparse
import json
import logging
import os
from pathlib import Path
import secrets
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--simulate', action='store_true', help='ROS-free simulation; no ROS topics are opened')
    parser.add_argument('--enable-motion', action='store_true', help='permit subsequent explicit browser arming')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default=18780, type=int)
    parser.add_argument('--data-dir', type=Path, default=Path('assets/navigation'))
    auth = parser.add_mutually_exclusive_group()
    auth.add_argument('--token-file', type=Path)
    auth.add_argument('--no-auth', action='store_true', help='serve the web UI and API without a control token')
    parser.add_argument('--ssl-certfile')
    parser.add_argument('--ssl-keyfile')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    config = json.loads(args.config.read_text(encoding='utf8'))
    args.data_dir.mkdir(parents=True, exist_ok=True)
    speed_file = args.data_dir/'speed-settings.json'
    if speed_file.exists(): config['speed_limits'] = json.loads(speed_file.read_text(encoding='utf8'))
    token_path = args.token_file or args.data_dir / 'control.token'
    token = None
    if not args.no_auth:
        if not token_path.exists():
            fd = os.open(token_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, 'w') as stream: stream.write(secrets.token_urlsafe(32))
        token = token_path.read_text().strip()
    from ...components.vehicle import TowingParameters
    from .service import NavigationService
    from .server import create_app
    if args.simulate:
        from .simulation import SimulationBridge
        bridge = SimulationBridge(TowingParameters(**config.get('vehicle', {})), config.get('drive_mode', 'towing'))
        config['required_scans'], config['optional_scans'] = ['scan'], []
        config['catalog'] = {}
        config['allow_mapping'] = False
    else:
        from ...adapters.ros1_local import LocalROS1Bridge
        parameters = TowingParameters(**config.get('vehicle', {}))
        config['ros']['drive_mode'] = config.get('drive_mode', 'towing')
        for key in ('min_turn_radius', 'rear_extent', 'rear_lateral_speed'):
            config['ros'][key] = getattr(parameters, key)
        bridge = LocalROS1Bridge(config['ros'], motion_enabled=args.enable_motion, **config.get('relay', {}))
    from ...runtime.sharing import SharedProvider
    from ...runtime.deployment import ProcessPlacement
    from ...runtime.options import RuntimeOptions, RuntimePaths
    from .client import NavigationClient
    controller = NavigationService(bridge, config, simulated=args.simulate, workers=True)
    runtime_options = RuntimeOptions(paths=RuntimePaths(log_root=args.data_dir.resolve() / 'workers'))
    provider = SharedProvider(controller, key='rsim-navigation:' + uuid.uuid4().hex,
                              interface_version='navigation-v2',
                              placement={controller.workers.local: ProcessPlacement('local-planning'),
                                         controller.workers.global_: ProcessPlacement('global-planning')},
                              log_path=args.data_dir / 'controller.log', options=runtime_options)
    service = NavigationClient(provider, config, simulated=args.simulate,
                               motion_enabled=bridge.motion_enabled)
    app = create_app(service, token, data_dir=args.data_dir,
                     allowed_hosts=config.get('allowed_hosts', ['localhost', '127.0.0.1']), runtime_options=runtime_options)
    logging.info('Authentication: %s; motion: %s', 'disabled' if args.no_auth else str(token_path.resolve()),
                 'simulation' if args.simulate else 'operator arm required' if args.enable_motion else 'ZERO ONLY')
    import uvicorn
    from threadpoolctl import threadpool_limits
    with threadpool_limits(limits=1):
        uvicorn.run(app, host=args.host, port=args.port, log_level='info', access_log=False,
                    ssl_certfile=args.ssl_certfile, ssl_keyfile=args.ssl_keyfile, ws_max_size=4096,
                    timeout_keep_alive=3, limit_concurrency=32)


if __name__ == '__main__': main()
