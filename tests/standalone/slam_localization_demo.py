#!/usr/bin/env python3
import math
import random

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import EKF2D, FourWheelForkliftModel, Pose2D, VehicleState
from warehouse_visual_localization.core.types import ControlCommand


def noisy(value: float, sigma: float) -> float:
    return value + random.gauss(0.0, sigma)


def main() -> None:
    args = parser("Standalone visual/wheel odometry localization demo").parse_args()
    vehicle, *_, ekf_cfg = load_vehicle_config(args.config)
    model = FourWheelForkliftModel(vehicle)
    ekf = EKF2D(Pose2D(), ekf_cfg)
    truth = VehicleState(Pose2D())
    out = ensure_output_dir(args.output_dir, "slam_localization")
    random.seed(7)

    rows = []
    for i in range(int(22.0 / args.dt)):
        t = i * args.dt
        cmd = ControlCommand(v=0.30, steer=0.38 * math.sin(0.35 * t))
        truth = model.step(truth, cmd, args.dt)
        wheel_v = noisy(truth.v, 0.015)
        wheel_w = noisy(truth.yaw_rate, 0.020)
        ekf.predict(wheel_v, wheel_w, args.dt)
        if i % 4 == 0:
            vision = Pose2D(
                noisy(truth.pose.x, 0.06),
                noisy(truth.pose.y, 0.06),
                noisy(truth.pose.yaw, 0.045),
            )
            ekf.update_visual_pose(vision)
        err = math.hypot(ekf.pose.x - truth.pose.x, ekf.pose.y - truth.pose.y)
        rows.append(
            {
                "t": t,
                "truth_x": truth.pose.x,
                "truth_y": truth.pose.y,
                "truth_yaw": truth.pose.yaw,
                "ekf_x": ekf.pose.x,
                "ekf_y": ekf.pose.y,
                "ekf_yaw": ekf.pose.yaw,
                "position_error": err,
            }
        )

    write_csv(out / "localization.csv", rows)
    mean_error = sum(r["position_error"] for r in rows) / len(rows)
    write_summary(out / "summary.json", {"samples": len(rows), "mean_position_error": mean_error, "final": rows[-1]})

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot([r["truth_x"] for r in rows], [r["truth_y"] for r in rows], label="truth")
    ax.plot([r["ekf_x"] for r in rows], [r["ekf_y"] for r in rows], label="ekf")
    ax.set_title("SLAM/Localization Standalone: wheel prediction + visual correction")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    fig.savefig(out / "localization.png", dpi=140)
    if args.show:
        plt.show()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

