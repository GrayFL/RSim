"""Offline diagnostic figures; uses the user's scipykit style when available."""
from pathlib import Path
import numpy as np


def diagnostic(sim, scan, frame, trajectory, output):
    import matplotlib
    matplotlib.use("Agg")
    try:
        from scipykit.task.cv import plt, subplots
    except ImportError:
        from matplotlib import pyplot as plt
        subplots = plt.subplots
    fig, axes = subplots(2, 2, figsize=(12, 9), layout="constrained")
    axes[0, 0].imshow(frame.rgb)
    axes[0, 0].set_title(f"RGB | t = {frame.time:.2f} s")
    axes[0, 0].axis("off")
    im = axes[0, 1].imshow(frame.depth, cmap="turbo", vmin=0, vmax=sim.sensors.depth_max)
    axes[0, 1].set_title("Optical depth [m]")
    axes[0, 1].axis("off")
    fig.colorbar(im, ax=axes[0, 1], shrink=0.8)
    points = scan.points()
    axes[1, 0].scatter(points[:, 0], points[:, 1], s=3)
    axes[1, 0].add_patch(plt.Rectangle((-.3, -.3), .6, .6, fill=False, color="tab:red"))
    axes[1, 0].arrow(0, 0, .8, 0, width=.025, color="tab:red")
    axes[1, 0].set(xlabel="Forward x [m]", ylabel="Left y [m]", title="360 degree lidar | z = 0.4 m", aspect="equal")
    axes[1, 0].grid(alpha=.2)
    nav = sim.navigation
    grid = np.zeros(nav["floor"].shape)
    grid[nav["floor"]] = 1
    grid[nav["obstacles"]] = .4
    origin, resolution = nav["origin"], float(nav["resolution"])
    upper = origin + np.array(grid.shape)*resolution
    axes[1, 1].imshow(grid.T, origin="lower", extent=[origin[0], upper[0], origin[1], upper[1]], cmap="gray", vmin=0, vmax=1)
    traj = np.asarray(trajectory)
    if len(traj):
        axes[1, 1].plot(traj[:, 1], traj[:, 2], color="tab:orange", linewidth=2)
    pose = sim.pose2d
    axes[1, 1].plot(pose[0], pose[1], "o", color="tab:red")
    axes[1, 1].set(xlabel="World x [m]", ylabel="World y [m]", title="Collision projection + ground-truth trajectory", aspect="equal")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor="white")
    plt.close(fig)

