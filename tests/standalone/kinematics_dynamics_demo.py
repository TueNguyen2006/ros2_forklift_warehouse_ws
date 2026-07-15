#!/usr/bin/env python3
import math

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import ControlCommand, FourWheelForkliftModel, Pose2D, VehicleState


def main() -> None:
    args = parser("2D forklift kinematics/dynamics demo").parse_args()
    vehicle, *_ = load_vehicle_config(args.config)
    model = FourWheelForkliftModel(vehicle)
    out = ensure_output_dir(args.output_dir, "kinematics_dynamics")

    state = VehicleState(Pose2D())
    rows = []
    for i in range(int(18.0 / args.dt)):
        t = i * args.dt
        steer = 0.45 * math.sin(0.45 * t) if t > 3.0 else 0.0
        cmd = ControlCommand(v=0.30, steer=steer)
        state = model.step(state, cmd, args.dt)
        forces = model.estimate_forces(state)
        rows.append(
            {
                "t": t,
                "x": state.pose.x,
                "y": state.pose.y,
                "yaw": state.pose.yaw,
                "v": state.v,
                "steer": state.steer,
                "yaw_rate": state.yaw_rate,
                "force_x": forces.longitudinal,
                "force_y": forces.lateral,
                "normal_front": forces.normal_front,
                "normal_rear": forces.normal_rear,
            }
        )

    write_csv(out / "trajectory.csv", rows)
    write_summary(out / "summary.json", {"samples": len(rows), "final": rows[-1]})

    import matplotlib.pyplot as plt

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, label="trajectory")
    ax.scatter([xs[0]], [ys[0]], label="start")
    ax.scatter([xs[-1]], [ys[-1]], label="final")
    ax.set_title("Forklift 2D Kinematics/Dynamics")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    fig.savefig(out / "trajectory.png", dpi=140)
    if args.show:
        plt.show()

    print(f"wrote {out}")


if __name__ == "__main__":
    main()

