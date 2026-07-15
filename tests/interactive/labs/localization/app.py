#!/usr/bin/env python3
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import button, parser, setup_figure, show_or_smoke, slider
from warehouse_visual_localization.core import ControlCommand, EKF2D, FourWheelForkliftModel, Pose2D, VehicleState


def main() -> None:
    args = parser("Interactive SLAM/localization lab").parse_args()
    fig, ax = setup_figure("SLAM/Localization Lab")
    vision_noise = slider(fig, 0, "vision noise", 0.0, 0.30, 0.06)
    wheel_noise = slider(fig, 1, "wheel noise", 0.0, 0.12, 0.02)
    steer_amp = slider(fig, 2, "steer amp", 0.0, 0.60, 0.35)
    step_btn = button(fig, 0, "Step")
    reset_btn = button(fig, 1, "Reset")
    model = FourWheelForkliftModel()
    truth = VehicleState(Pose2D())
    ekf = EKF2D(Pose2D())
    truth_trail: list[tuple[float, float]] = []
    ekf_trail: list[tuple[float, float]] = []
    tick = 0
    random.seed(3)

    def redraw() -> None:
        ax.clear()
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.set_xlim(-1, 8)
        ax.set_ylim(-4, 4)
        if truth_trail:
            ax.plot([p[0] for p in truth_trail], [p[1] for p in truth_trail], "k-", label="truth")
            ax.plot([p[0] for p in ekf_trail], [p[1] for p in ekf_trail], "r--", label="ekf")
        err = math.hypot(ekf.pose.x - truth.pose.x, ekf.pose.y - truth.pose.y)
        ax.set_title(f"EKF visual correction | error={err:.3f} m | vision_noise={vision_noise.val:.2f}")
        if truth_trail:
            ax.legend()
        fig.canvas.draw_idle()

    def step(_event=None) -> None:
        nonlocal truth, tick
        for _ in range(12):
            t = tick * 0.05
            truth = model.step(truth, ControlCommand(v=0.30, steer=steer_amp.val * math.sin(0.35 * t)), 0.05)
            ekf.predict(
                truth.v + random.gauss(0.0, wheel_noise.val),
                truth.yaw_rate + random.gauss(0.0, wheel_noise.val),
                0.05,
            )
            if tick % 4 == 0:
                ekf.update_visual_pose(
                    Pose2D(
                        truth.pose.x + random.gauss(0.0, vision_noise.val),
                        truth.pose.y + random.gauss(0.0, vision_noise.val),
                        truth.pose.yaw + random.gauss(0.0, vision_noise.val),
                    )
                )
            truth_trail.append((truth.pose.x, truth.pose.y))
            ekf_trail.append((ekf.pose.x, ekf.pose.y))
            tick += 1
        redraw()

    def reset(_event=None) -> None:
        nonlocal truth, ekf, truth_trail, ekf_trail, tick
        truth = VehicleState(Pose2D())
        ekf = EKF2D(Pose2D())
        truth_trail = []
        ekf_trail = []
        tick = 0
        redraw()

    step_btn.on_clicked(step)
    reset_btn.on_clicked(reset)
    redraw()
    if args.smoke:
        step()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
