#!/usr/bin/env python3
import math

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import FourWheelForkliftModel, Pose2D, PurePursuitController, VehicleState


def make_path() -> list[Pose2D]:
    path = []
    for i in range(80):
        x = i * 0.08
        y = 0.7 * math.sin(0.8 * x)
        yaw = math.atan2(0.56 * math.cos(0.8 * x), 1.0)
        path.append(Pose2D(x, y, yaw))
    return path


def main() -> None:
    args = parser("Controller and trajectory tracking standalone demo").parse_args()
    vehicle, controller_cfg, *_ = load_vehicle_config(args.config)
    controller = PurePursuitController(controller_cfg, vehicle)
    model = FourWheelForkliftModel(vehicle)
    out = ensure_output_dir(args.output_dir, "controller_tracking")
    state = VehicleState(Pose2D(0.0, -0.45, 0.0))
    path = make_path()
    rows = []
    for i in range(int(32.0 / args.dt)):
        t = i * args.dt
        cmd = controller.compute(state, path)
        state = model.step(state, cmd, args.dt)
        nearest_error = min(math.hypot(state.pose.x - p.x, state.pose.y - p.y) for p in path)
        rows.append(
            {
                "t": t,
                "x": state.pose.x,
                "y": state.pose.y,
                "yaw": state.pose.yaw,
                "v": state.v,
                "steer": state.steer,
                "tracking_error": nearest_error,
            }
        )
        if cmd.v == 0.0:
            break
    write_csv(out / "controller_tracking.csv", rows)
    write_summary(out / "summary.json", {"samples": len(rows), "mean_tracking_error": sum(r["tracking_error"] for r in rows) / len(rows), "final": rows[-1]})

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot([p.x for p in path], [p.y for p in path], label="reference")
    ax.plot([r["x"] for r in rows], [r["y"] for r in rows], label="tracked")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Standalone Controller Tracking")
    ax.grid(True)
    ax.legend()
    fig.savefig(out / "controller_tracking.png", dpi=140)
    if args.show:
        plt.show()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

