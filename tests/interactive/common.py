#!/usr/bin/env python3
import argparse
import math
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKSPACE_ROOT / "src" / "warehouse_visual_localization"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from warehouse_visual_localization.core import Pose2D, VehicleState  # noqa: E402


def parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--smoke", action="store_true", help="Run a non-interactive sanity check.")
    return p


def setup_figure(title: str):
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 7))
    fig.canvas.manager.set_window_title(title)
    ax = fig.add_axes([0.06, 0.10, 0.62, 0.82])
    panel = fig.add_axes([0.72, 0.08, 0.25, 0.86])
    panel.axis("off")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    return fig, ax


def slider(fig, index: int, label: str, vmin: float, vmax: float, value: float):
    from matplotlib.widgets import Slider

    y = 0.87 - index * 0.055
    ax = fig.add_axes([0.74, y, 0.20, 0.025])
    return Slider(ax, label, vmin, vmax, valinit=value)


def button(fig, index: int, label: str):
    from matplotlib.widgets import Button

    y = 0.14 - index * 0.055
    ax = fig.add_axes([0.74, y, 0.09, 0.035])
    return Button(ax, label)


def textbox(fig, index: int, label: str, text: str):
    from matplotlib.widgets import TextBox

    y = 0.30 - index * 0.055
    ax = fig.add_axes([0.74, y, 0.20, 0.035])
    return TextBox(ax, label, initial=text)


def transform_points(points: list[tuple[float, float]], pose: Pose2D) -> list[tuple[float, float]]:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return [(pose.x + c * x - s * y, pose.y + s * x + c * y) for x, y in points]


def draw_forklift(ax, state: VehicleState, color: str = "#d79a21") -> None:
    body = [(0.35, 0.50), (0.35, -0.50), (-1.10, -0.50), (-1.10, 0.50), (0.35, 0.50)]
    forks = [(0.35, 0.18), (0.95, 0.18), (0.35, -0.18), (0.95, -0.18)]
    wheels = [(-0.70, 0.43), (-0.70, -0.43), (0.10, 0.43), (0.10, -0.43)]
    body_w = transform_points(body, state.pose)
    forks_w = transform_points(forks, state.pose)
    ax.plot([p[0] for p in body_w], [p[1] for p in body_w], color=color, linewidth=2)
    ax.fill([p[0] for p in body_w[:-1]], [p[1] for p in body_w[:-1]], color=color, alpha=0.25)
    ax.plot([forks_w[0][0], forks_w[1][0]], [forks_w[0][1], forks_w[1][1]], color="gray", linewidth=4)
    ax.plot([forks_w[2][0], forks_w[3][0]], [forks_w[2][1], forks_w[3][1]], color="gray", linewidth=4)
    for wx, wy in transform_points(wheels, state.pose):
        ax.add_patch(plt_circle((wx, wy), 0.07, "black"))
    ax.arrow(state.pose.x, state.pose.y, 0.45 * math.cos(state.pose.yaw), 0.45 * math.sin(state.pose.yaw), width=0.015, color="#105bd8")
    ax.scatter([state.pose.x], [state.pose.y], color="red", s=35, label="CG")


def plt_circle(center: tuple[float, float], radius: float, color: str):
    import matplotlib.patches as patches

    return patches.Circle(center, radius, color=color, alpha=0.9)


def show_or_smoke(fig, smoke: bool) -> None:
    import matplotlib.pyplot as plt

    if smoke:
        fig.canvas.draw()
        plt.close(fig)
    else:
        plt.show()

