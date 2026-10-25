"""ROS-free closed-loop validation and optional scipykit plot of towing paths."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np
from threadpoolctl import threadpool_limits
from pyrobo import Bounds2D
from rsim.components.towing_geometry import BodyFootprint, advance, wrap
from rsim.components.towing_navigation import TowingNavigator


def run(name, rear, start, goal, points, output, drive_mode='towing'):
    nav = TowingNavigator(footprint=BodyFootprint(rear=rear), drive_mode=drive_mode)
    bounds = Bounds2D(-6, 6, -6, 6)
    obstacles = nav.obstacle_index(points, 'map', .02)
    state, measured = np.array(start, float), np.zeros(2)
    rows, timings = [], []
    for tick in range(1800):
        started = time.perf_counter()
        decision = nav.step(state, measured, goal, obstacles, bounds)
        timings.append((time.perf_counter() - started) * 1000)
        assert decision.safe, (name, tick, decision.status)
        assert obstacles.path_safe(decision.trajectory, nav.footprint, bounds)
        command = np.array([decision.command.linear_x, decision.command.angular_z])
        updated = nav.model.advance_velocity(measured, command, .1)
        assert abs(updated[0]) <= 1.6 + 1e-8
        if drive_mode == 'towing':
            assert updated[0] >= .3 * abs(updated[1]) - 1e-8
            assert abs(updated[1]) * .81 <= .12 + 1e-8
        rows.append([tick * .1, *state, *updated])
        midpoint = (measured + updated) / 2
        state, measured = advance(state, midpoint[0], midpoint[1], .1), updated
        if decision.status == 'arrived': break
    assert decision.status == 'arrived', (name, state.tolist())
    rows = np.asarray(rows)
    with (output / (name + '.csv')).open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'x_m', 'y_m', 'yaw_rad', 'v_m_s', 'w_rad_s'])
        writer.writerows(rows)
    metrics = dict(name=name, rear=rear, drive_mode=drive_mode, simulated_seconds=round(rows[-1, 0], 1),
        distance_m=round(float(np.linalg.norm(np.diff(rows[:, 1:3], axis=0), axis=1).sum()), 3),
        position_error_m=round(float(np.linalg.norm(state[:2] - goal[:2])), 3),
        heading_error_rad=round(float(abs(wrap(state[2] - goal[2]))), 3),
        maximum_rear_lateral_m_s=round(float(np.max(abs(rows[:, 5])) * .81), 4),
        min_forward_speed_m_s=float(rows[:, 4].min()),
        compute_p95_ms=round(float(np.percentile(timings[1:], 95)), 2),
        first_plan_ms=round(timings[0], 2), status=decision.status)
    return rows, metrics, nav.footprint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('assets/towing-validation'))
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--plot-only', action='store_true', help='redraw existing CSV/metrics without rerunning validation')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    wall = np.c_[np.zeros(61), np.linspace(-1.2, 1.2, 61)]
    scenarios = [('turn_retracted', .27, [0, 0, 0], [-1.5, 0, np.pi], np.empty((0, 2))),
                 ('turn_extended', .81, [0, 0, 0], [-1.5, 0, np.pi], np.empty((0, 2))),
                 ('wall_detour', .81, [-2, 0, 0], [2, 0, 0], wall)]
    if args.plot_only:
        previous = json.loads((args.output / 'metrics.json').read_text())
        results = [(np.loadtxt(args.output / (item['name'] + '.csv'), delimiter=',', skiprows=1),
                    item, BodyFootprint(rear=item['rear'])) for item in previous]
    else:
        with threadpool_limits(limits=1):
            results = [run(*scenario, args.output) for scenario in scenarios]
    metrics = [item[1] for item in results]
    (args.output / 'metrics.json').write_text(json.dumps(metrics, indent=2))
    if args.plot or args.plot_only:
        from scipykit.mtp_initializer import subplots, plt
        fig, axes = subplots(1, 3, figsize=(14, 4.5))
        for ax, scenario, result in zip(axes, scenarios, results):
            rows, item, body = result
            ax.plot(rows[:, 1], rows[:, 2], color='#12856b', linewidth=2)
            points = scenario[-1]
            if len(points): ax.plot(points[:, 0], points[:, 1], color='#b55141', linewidth=4)
            for pose in rows[::max(1, len(rows) // 10), 1:4]:
                corners = body.corners(pose)
                closed = np.vstack((corners, corners[0]))
                ax.plot(closed[:, 0], closed[:, 1], color='#12856b', alpha=.25, linewidth=.8)
            ax.scatter(*scenario[2][:2], color='#336bba', s=35, label='Start')
            ax.scatter(*scenario[3][:2], marker='x', color='#cc7b12', s=55, label='Goal')
            ax.set_title(item['name'].replace('_', ' ') + '\n' + str(item['distance_m']) + ' m, ' + str(item['simulated_seconds']) + ' s')
            ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
            ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend()
        fig.tight_layout()
        fig.savefig(args.output / 'trajectories.png', dpi=160, bbox_inches='tight', pad_inches=.2)
        plt.close(fig)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__': main()
