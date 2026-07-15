#!/usr/bin/env python3
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import button, parser, setup_figure, show_or_smoke, slider
from warehouse_visual_localization.core import Pose2D, VehicleState
from warehouse_visual_localization.core.math_utils import wrap_angle
from warehouse_visual_localization.core.wheel_kinematics import FourWheelKinematics


def main() -> None:
    args = parser("Interactive wheel-level forklift kinematics lab").parse_args()
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    kin = FourWheelKinematics()
    state = VehicleState(Pose2D())
    trail: list[tuple[float, float]] = []
    fig, ax = setup_figure("Wheel-Level Forklift Kinematics Lab")

    speed_fl = slider(fig, 0, "FL speed", -0.6, 0.6, 0.25)
    speed_fr = slider(fig, 1, "FR speed", -0.6, 0.6, 0.25)
    speed_rl = slider(fig, 2, "RL speed", -0.6, 0.6, 0.25)
    speed_rr = slider(fig, 3, "RR speed", -0.6, 0.6, 0.25)
    steer_r = slider(fig, 4, "rear steer", -0.75, 0.75, 0.35)
    ideal_v = slider(fig, 5, "ideal vx", -0.18, 0.45, 0.25)
    dt_slider = slider(fig, 6, "dt", 0.01, 0.20, 0.05)
    step_count = slider(fig, 7, "steps", 1, 80, 10)
    step_btn = button(fig, 0, "Step")
    reset_btn = button(fig, 1, "Reset")
    ideal_btn = button(fig, 2, "Ideal")

    def solve():
        return kin.rear_steer_twist_from_wheels(
            speed_fl.val,
            speed_fr.val,
            speed_rl.val,
            speed_rr.val,
            steer_r.val,
        )

    def transform(x: float, y: float) -> tuple[float, float]:
        c = math.cos(state.pose.yaw)
        s = math.sin(state.pose.yaw)
        return state.pose.x + c * x - s * y, state.pose.y + s * x + c * y

    def draw_vehicle(result) -> None:
        body = [transform(0.45, 0.50), transform(0.45, -0.50), transform(-1.25, -0.50), transform(-1.25, 0.50)]
        ax.fill([p[0] for p in body], [p[1] for p in body], color="#d79a21", alpha=0.25)
        ax.plot([* [p[0] for p in body], body[0][0]], [* [p[1] for p in body], body[0][1]], color="#d79a21", linewidth=2)
        wheel_len = 0.28
        wheel_w = 0.08
        front_ref = transform(0.0, 0.0)
        cg = transform(-0.55, 0.0)
        speed_by_name = {
            "front_left": speed_fl.val,
            "front_right": speed_fr.val,
            "rear_left": speed_rl.val,
            "rear_right": speed_rr.val,
        }
        for name, (rx, ry) in kin.positions.items():
            # Convert from the core's centered wheel map to front-axle reference.
            rx = rx - kin.positions["front_left"][0]
            wheel_speed = speed_by_name[name]
            wx, wy = transform(rx, ry)
            steer = steer_r.val if name.startswith("rear") else 0.0
            yaw = state.pose.yaw + steer
            rect = Rectangle((-wheel_len / 2, -wheel_w / 2), wheel_len, wheel_w, angle=0, color="black")
            rect.set_transform(
                plt.matplotlib.transforms.Affine2D()
                .rotate(yaw)
                .translate(wx, wy)
                + ax.transData
            )
            ax.add_patch(rect)
            ax.arrow(wx, wy, 0.35 * wheel_speed * math.cos(yaw), 0.35 * wheel_speed * math.sin(yaw), color="green", width=0.01)
            ax.text(wx, wy + 0.12, name.replace("_", "\n"), fontsize=7, ha="center")
        # The front axle is nonholonomic, so its local velocity vector is always
        # along the chassis x-axis. Turning is shown by yaw-rate arc/vector.
        front_vx_world = result.vx * math.cos(state.pose.yaw)
        front_vy_world = result.vx * math.sin(state.pose.yaw)
        ax.arrow(
            front_ref[0],
            front_ref[1],
            front_vx_world,
            front_vy_world,
            color="blue",
            width=0.015,
            label="front axle velocity",
        )
        turn_sign = 1.0 if result.yaw_rate >= 0.0 else -1.0
        arc_radius = 0.55
        ax.arrow(
            front_ref[0],
            front_ref[1],
            -turn_sign * arc_radius * math.sin(state.pose.yaw),
            turn_sign * arc_radius * math.cos(state.pose.yaw),
            color="purple",
            width=0.012,
            label="yaw-rate direction",
        )
        if abs(result.yaw_rate) > 1e-5:
            radius = result.vx / result.yaw_rate
            icr_body = (0.0, radius)
            icr_world = transform(*icr_body)
            ax.scatter([icr_world[0]], [icr_world[1]], color="cyan", s=45, label="ICR")
            ax.plot([front_ref[0], icr_world[0]], [front_ref[1], icr_world[1]], color="cyan", linestyle="--", linewidth=1)
        ax.scatter([front_ref[0]], [front_ref[1]], color="red", s=35, label="front axle ref")
        ax.scatter([cg[0]], [cg[1]], color="purple", s=35, label="CG approx")

    def integrate(result, dt: float) -> None:
        c = math.cos(state.pose.yaw)
        s = math.sin(state.pose.yaw)
        state.pose.x += (c * result.vx - s * result.vy) * dt
        state.pose.y += (s * result.vx + c * result.vy) * dt
        state.pose.yaw = wrap_angle(state.pose.yaw + result.yaw_rate * dt)
        state.v = result.vx
        state.yaw_rate = result.yaw_rate
        trail.append((state.pose.x, state.pose.y))

    def redraw() -> None:
        result = solve()
        ax.clear()
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.set_xlim(-2.0, 8.0)
        ax.set_ylim(-4.0, 4.0)
        if trail:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], "r-", label="trajectory")
        draw_vehicle(result)
        color = "green" if result.residual < 0.015 else "red"
        ax.set_title(
            "Rear-steer forklift kinematics: front wheels fixed, rear wheels steer, vy_front=0\n"
            f"vx_front={result.vx:.3f} yaw_rate={result.yaw_rate:.3f} "
            f"turn_radius={(result.vx / result.yaw_rate if abs(result.yaw_rate) > 1e-5 else float('inf')):.2f} "
            f"rear_steer={steer_r.val:.3f} residual={result.residual:.4f}"
        )
        ax.text(-1.8, 3.45, "residual xanh=consistent, đỏ=wheel speeds conflict", color=color, fontsize=10)
        ax.legend(loc="upper left")
        fig.canvas.draw_idle()

    def step(_event=None) -> None:
        for _ in range(int(step_count.val)):
            result = solve()
            integrate(result, dt_slider.val)
        redraw()

    def reset(_event=None) -> None:
        nonlocal state, trail
        state = VehicleState(Pose2D())
        trail = []
        redraw()

    def set_ideal(_event=None) -> None:
        speeds = kin.ideal_rear_steer_wheel_speeds(ideal_v.val, steer_r.val)
        speed_fl.set_val(speeds["front_left"])
        speed_fr.set_val(speeds["front_right"])
        speed_rl.set_val(speeds["rear_left"])
        speed_rr.set_val(speeds["rear_right"])
        redraw()

    step_btn.on_clicked(step)
    reset_btn.on_clicked(reset)
    ideal_btn.on_clicked(set_ideal)
    for widget in [speed_fl, speed_fr, speed_rl, speed_rr, steer_r, ideal_v, dt_slider, step_count]:
        widget.on_changed(lambda _v: redraw())
    redraw()
    if args.smoke:
        step()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
