#!/usr/bin/env python3
"""Visualize curvature-limited v_max directly on a planned forklift path."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "warehouse_visual_localization"))
sys.path.insert(0, str(ROOT / "tests" / "interactive"))
from common import button, draw_forklift, parser, setup_figure, show_or_smoke  # noqa: E402
from warehouse_visual_localization.core import Pose2D, VehicleState  # noqa: E402
from warehouse_visual_localization.core.curvature_speed import acceleration_limited_speed_profile, CurvatureSpeedConfig, curvature_speed_limit, signed_curvature  # noqa: E402


def make_path() -> np.ndarray:
    """Straight -> curvature-ramp turn -> straight, integrated by arc length.

    The entry/exit ramps approximate clothoids. Unlike a raw line-to-circle
    join, curvature is continuous, so a speed scheduler has room to brake
    progressively before the constant-radius part of the turn.
    """
    step = 0.02
    radius = 0.45
    kappa_peak = 1.0 / radius
    transition_length = 0.40
    transition_heading = 0.5 * kappa_peak * transition_length
    constant_length = (np.pi / 2.0 - 2.0 * transition_heading) / kappa_peak
    samples: list[tuple[float, float]] = []
    x, y, heading = -3.0, 0.0, 0.0
    schedule = [(2.2, lambda _: 0.0), (transition_length, lambda q: kappa_peak * q), (constant_length, lambda _: kappa_peak), (transition_length, lambda q: kappa_peak * (1.0 - q)), (2.5, lambda _: 0.0)]
    for length, curvature_fn in schedule:
        for distance in np.arange(0.0, length, step):
            samples.append((x, y))
            kappa = curvature_fn(distance / max(length, step))
            heading += kappa * step
            x += np.cos(heading) * step
            y += np.sin(heading) * step
    samples.append((x, y))
    return np.asarray(samples, dtype=float)


def main() -> None:
    args = parser("Curvature-speed profile lab").parse_args()
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    from matplotlib.widgets import Button, Slider

    path = make_path()
    curvature = signed_curvature(path)
    progress = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    fig = plt.figure(figsize=(15, 8))
    fig.canvas.manager.set_window_title("Curvature-Speed Profile on Planned Path")
    ax = fig.add_axes([.05, .12, .53, .78])
    control_title = fig.text(.71, .91, "Controls", fontsize=13, fontweight="bold")
    speed_cap_slider = Slider(fig.add_axes([.71, .83, .23, .028]), "vehicle speed cap", .10, .50, valinit=.32)
    lateral_slider = Slider(fig.add_axes([.71, .74, .23, .028]), "lateral accel limit", .05, 1.00, valinit=.10)
    requested_slider = Slider(fig.add_axes([.71, .65, .23, .028]), "requested speed", .05, .50, valinit=.32)
    play = Button(fig.add_axes([.71, .48, .11, .05]), "Play")
    run_demo = Button(fig.add_axes([.84, .48, .11, .05]), "Run demo")
    step = Button(fig.add_axes([.71, .40, .11, .05]), "Step")
    reset = Button(fig.add_axes([.84, .40, .11, .05]), "Reset")
    state = {"distance": 0.0, "speed": 0.0, "playing": False}
    colorbar_ax = fig.add_axes([.61, .18, .018, .60])
    artists = {"body": None, "fork_a": None, "fork_b": None, "cg": None, "progress": None}

    def profile():
        local_cap = curvature_speed_limit(curvature, CurvatureSpeedConfig(vehicle_speed_limit=speed_cap_slider.val, lateral_acceleration_limit=lateral_slider.val)).speed_limit
        return acceleration_limited_speed_profile(local_cap, progress, .55, .65)

    def set_vehicle(index, vmax):
        from matplotlib.patches import Polygon
        x = float(np.interp(state["distance"], progress, path[:, 0])); y = float(np.interp(state["distance"], progress, path[:, 1]))
        dx = np.gradient(path[:, 0], progress); dy = np.gradient(path[:, 1], progress)
        heading = np.arctan2(np.interp(state["distance"], progress, dy), np.interp(state["distance"], progress, dx))
        c, s = np.cos(heading), np.sin(heading)
        def transform(points):
            points = np.asarray(points)
            return np.column_stack((x + c * points[:,0] - s * points[:,1], y + s * points[:,0] + c * points[:,1]))
        body = transform(np.array([[.35,.5],[.35,-.5],[-1.1,-.5],[-1.1,.5]]))
        forks = transform(np.array([[.35,.18],[.95,.18],[.35,-.18],[.95,-.18]]))
        if artists["body"] is None:
            artists["body"] = Polygon(body, closed=True, facecolor="#d79a21", edgecolor="#b57900", alpha=.35, linewidth=2, zorder=4)
            ax.add_patch(artists["body"])
            artists["fork_a"], = ax.plot([], [], color="gray", linewidth=4, zorder=4)
            artists["fork_b"], = ax.plot([], [], color="gray", linewidth=4, zorder=4)
            artists["cg"], = ax.plot([], [], "o", color="red", label="CG", zorder=5)
            artists["progress"], = ax.plot([], [], "o", color="blue", label="forklift progress", zorder=5)
            ax.legend(loc="upper left")
        artists["body"].set_xy(body)
        artists["fork_a"].set_data(forks[:2,0], forks[:2,1]); artists["fork_b"].set_data(forks[2:,0], forks[2:,1])
        artists["cg"].set_data([x], [y]); artists["progress"].set_data([x], [y])
        ax.set_title(f"Braking-aware speed profile: progress={state['distance']:.2f} m | curvature={curvature[index]:.2f} 1/m | requested={requested_slider.val:.2f} | applied={state['speed']:.2f} m/s")

    def redraw_static():
        vmax = profile()
        ax.clear(); artists.update(body=None, fork_a=None, fork_b=None, cg=None, progress=None); ax.set_aspect("equal", adjustable="box"); ax.grid(True, alpha=.35)
        segments = np.stack((path[:-1], path[1:]), axis=1)
        collection = LineCollection(segments, cmap="RdYlGn", norm=Normalize(vmin=0.0, vmax=speed_cap_slider.val))
        collection.set_array(vmax[:-1]); collection.set_linewidth(6); ax.add_collection(collection)
        colorbar_ax.clear()
        colorbar = fig.colorbar(collection, cax=colorbar_ax)
        colorbar.set_label("v_ref(s) [m/s]: red = slow, green = fast")
        index = min(int(np.searchsorted(progress, state["distance"], side="right")), len(path) - 1)
        ax.set_xlim(-3.8, .8); ax.set_ylim(-1.0, 4.0)
        set_vehicle(index, vmax)
        fig.canvas.draw_idle()

    def redraw_vehicle():
        vmax = profile()
        index = min(int(np.searchsorted(progress, state["distance"], side="right")), len(path) - 1)
        set_vehicle(index, vmax)
        fig.canvas.draw_idle()

    def advance(_=None):
        index = min(int(np.searchsorted(progress, state["distance"], side="right")), len(path) - 1)
        target_speed = min(requested_slider.val, profile()[index])
        dt = .03
        delta = target_speed - state["speed"]
        max_delta = (.55 if delta >= 0.0 else .65) * dt
        state["speed"] += np.clip(delta, -max_delta, max_delta)
        state["distance"] = min(progress[-1], state["distance"] + state["speed"] * dt)
        redraw_vehicle()

    def toggle(_=None):
        state["playing"] = not state["playing"]
        play.label.set_text("Pause" if state["playing"] else "Play")

    def restart(_=None):
        state.update(distance=0.0, speed=0.0, playing=False); play.label.set_text("Play"); redraw_vehicle()

    def start_demo(_=None):
        state.update(distance=0.0, speed=0.0, playing=True)
        play.label.set_text("Pause")
        redraw_vehicle()

    timer = fig.canvas.new_timer(interval=30); timer.add_callback(lambda: advance() if state["playing"] else None); timer.start()
    play.on_clicked(toggle); run_demo.on_clicked(start_demo); step.on_clicked(advance); reset.on_clicked(restart)
    for widget in (speed_cap_slider, lateral_slider, requested_slider): widget.on_changed(lambda _: redraw_static())
    redraw_static()
    if args.smoke: advance()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__": main()
