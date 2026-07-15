#!/usr/bin/env python3
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import button, draw_forklift, parser, setup_figure, show_or_smoke, slider
from warehouse_visual_localization.core import ControlCommand, FourWheelForkliftModel, Pose2D, VehicleState
from warehouse_visual_localization.core.math_utils import clamp, wrap_angle
from warehouse_visual_localization.core.safety import PolygonCBF2D


def main() -> None:
    args = parser("Interactive animated octagonal CBF forklift lab").parse_args()
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    model = FourWheelForkliftModel()
    state = VehicleState(Pose2D(-2.2, -1.8, 0.0))
    reference = [3.8, 2.8]
    trail: list[tuple[float, float]] = []
    playing = {"enabled": False}

    fig, ax = setup_figure("Animated Octagonal CBF Forklift Lab")
    limit = slider(fig, 0, "box limit", 1.5, 5.0, 3.2)
    diagonal = slider(fig, 1, "diag limit", 2.5, 7.5, 4.9)
    gamma = slider(fig, 2, "CBF gamma", 0.2, 5.0, 2.0)
    gain = slider(fig, 3, "goal gain", 0.1, 2.5, 0.85)
    vmax = slider(fig, 4, "v max", 0.05, 0.8, 0.38)
    yaw_gain = slider(fig, 5, "yaw gain", 0.3, 4.0, 1.8)
    dt_slider = slider(fig, 6, "dt", 0.01, 0.15, 0.04)
    step_count = slider(fig, 7, "substeps", 1, 30, 4)
    play_btn = button(fig, 0, "Play")
    step_btn = button(fig, 1, "Step")
    reset_btn = button(fig, 2, "Reset")

    def cbf() -> PolygonCBF2D:
        return PolygonCBF2D.octagon(limit.val, diagonal.val, gamma.val)

    def polygon_vertices() -> list[tuple[float, float]]:
        l = limit.val
        d = max(diagonal.val, 1.42 * l)
        return [
            (l, d - l),
            (d - l, l),
            (-(d - l), l),
            (-l, d - l),
            (-l, -(d - l)),
            (-(d - l), -l),
            (d - l, -l),
            (l, -(d - l)),
        ]

    def desired_velocity() -> tuple[float, float]:
        ux = gain.val * (reference[0] - state.pose.x)
        uy = gain.val * (reference[1] - state.pose.y)
        mag = math.hypot(ux, uy)
        if mag > vmax.val:
            ux *= vmax.val / mag
            uy *= vmax.val / mag
        return ux, uy

    def command_from_safe_velocity(vx: float, vy: float) -> ControlCommand:
        speed = min(math.hypot(vx, vy), vmax.val)
        if speed < 1e-4:
            return ControlCommand()
        desired_heading = math.atan2(vy, vx)
        heading_error = wrap_angle(desired_heading - state.pose.yaw)
        yaw_rate = clamp(yaw_gain.val * heading_error, -0.55, 0.55)
        steer = model.steer_from_yaw_rate(max(speed, 0.05), yaw_rate)
        return ControlCommand(v=speed, yaw_rate=yaw_rate, steer=steer)

    def safe_step(raw_u: tuple[float, float], dt: float) -> None:
        nonlocal state
        current_h = cbf().min_h(state.pose.x, state.pose.y)
        safe_u = cbf().filter_velocity(state.pose.x, state.pose.y, raw_u[0], raw_u[1])
        cmd = command_from_safe_velocity(*safe_u)

        # CBF is defined on point-mass velocity. The forklift has heading and
        # steering lag, so validate the actual kinematic step and backtrack the
        # command if it would reduce the barrier too aggressively.
        scale = 1.0
        accepted = None
        for _ in range(12):
            candidate_cmd = ControlCommand(
                v=cmd.v * scale,
                yaw_rate=cmd.yaw_rate * scale,
                steer=cmd.steer * scale,
            )
            candidate = model.step(state, candidate_cmd, dt)
            next_h = cbf().min_h(candidate.pose.x, candidate.pose.y)
            if next_h >= -1e-3 and next_h >= current_h - 0.04:
                accepted = candidate
                break
            scale *= 0.5

        if accepted is None:
            accepted = VehicleState(Pose2D(state.pose.x, state.pose.y, state.pose.yaw))
        state = accepted

    def draw_constraints() -> None:
        view = 5.4
        ax.add_patch(
            Rectangle(
                (-view, -view),
                2.0 * view,
                2.0 * view,
                color="#ff6b6b",
                alpha=0.18,
                label="unsafe outside safe set",
            )
        )
        vertices = polygon_vertices()
        ax.add_patch(Polygon(vertices, closed=True, fill=True, color="white", alpha=0.95))
        ax.add_patch(
            Polygon(
                vertices,
                closed=True,
                fill=False,
                color="black",
                linewidth=2.5,
                label="safe octagon",
            )
        )

    def frame_update(redraw_only: bool = False) -> None:
        nonlocal state
        if not redraw_only:
            for _ in range(int(step_count.val)):
                raw_u = desired_velocity()
                safe_step(raw_u, dt_slider.val)
                trail.append((state.pose.x, state.pose.y))
                if len(trail) > 2500:
                    del trail[:500]
        redraw()

    def redraw() -> None:
        raw_u = desired_velocity()
        safe_u = cbf().filter_velocity(state.pose.x, state.pose.y, raw_u[0], raw_u[1])
        h_min = cbf().min_h(state.pose.x, state.pose.y)

        ax.clear()
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.45)
        ax.set_xlim(-5.4, 5.4)
        ax.set_ylim(-5.4, 5.4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        draw_constraints()

        if trail:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], color="#0b64c0", linewidth=2, label="trajectory")
        draw_forklift(ax, state)
        ax.scatter([reference[0]], [reference[1]], marker="*", color="green", s=220, label="clicked reference")
        ax.arrow(state.pose.x, state.pose.y, raw_u[0], raw_u[1], color="orange", width=0.025, label="desired velocity")
        ax.arrow(state.pose.x, state.pose.y - 0.18, safe_u[0], safe_u[1], color="purple", width=0.025, label="CBF velocity")

        status = "PLAY" if playing["enabled"] else "PAUSE"
        ax.set_title(
            "Animated CBF forklift: desired velocity is projected into the octagonal safe set\n"
            f"{status} | h_min={h_min:.3f} | raw=({raw_u[0]:.2f},{raw_u[1]:.2f}) "
            f"cbf=({safe_u[0]:.2f},{safe_u[1]:.2f}) | v={state.v:.2f} yaw_rate={state.yaw_rate:.2f}"
        )
        ax.legend(loc="upper left")
        fig.canvas.draw_idle()

    def on_timer() -> bool:
        if playing["enabled"]:
            frame_update(redraw_only=False)
        return True

    timer = fig.canvas.new_timer(interval=40)
    timer.add_callback(on_timer)
    timer.start()

    def toggle_play(_event=None) -> None:
        playing["enabled"] = not playing["enabled"]
        play_btn.label.set_text("Pause" if playing["enabled"] else "Play")
        redraw()

    def single_step(_event=None) -> None:
        frame_update(redraw_only=False)

    def reset(_event=None) -> None:
        nonlocal state, trail
        state = VehicleState(Pose2D(-2.2, -1.8, 0.0))
        trail = []
        playing["enabled"] = False
        play_btn.label.set_text("Play")
        redraw()

    def on_click(event) -> None:
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        reference[:] = [event.xdata, event.ydata]
        redraw()

    play_btn.on_clicked(toggle_play)
    step_btn.on_clicked(single_step)
    reset_btn.on_clicked(reset)
    for widget in [limit, diagonal, gamma, gain, vmax, yaw_gain, dt_slider, step_count]:
        widget.on_changed(lambda _v: redraw())
    fig.canvas.mpl_connect("button_press_event", on_click)

    redraw()
    if args.smoke:
        playing["enabled"] = True
        min_h_seen = cbf().min_h(state.pose.x, state.pose.y)
        for _ in range(40):
            frame_update(redraw_only=False)
            min_h_seen = min(min_h_seen, cbf().min_h(state.pose.x, state.pose.y))
        if min_h_seen < -1e-3:
            raise RuntimeError(f"CBF smoke test violated safe set: min_h={min_h_seen:.4f}")
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
