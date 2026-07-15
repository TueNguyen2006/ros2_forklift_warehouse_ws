#!/usr/bin/env python3

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import CBFSafetyLayer, ControlCommand, Pose2D, VehicleState


def main() -> None:
    args = parser("CBF safety layer standalone demo").parse_args()
    vehicle, _, safety_cfg, *_ = load_vehicle_config(args.config)
    layer = CBFSafetyLayer(safety_cfg, vehicle)
    out = ensure_output_dir(args.output_dir, "cbf_safety")
    state = VehicleState(Pose2D(), v=0.25)
    rows = []
    for i in range(80):
        dist = 2.2 - i * 0.025
        raw = ControlCommand(v=0.30, yaw_rate=0.40, steer=0.30)
        safe = layer.filter(state, raw, dist)
        rows.append(
            {
                "obstacle_distance": dist,
                "raw_v": raw.v,
                "safe_v": safe.v,
                "raw_yaw_rate": raw.yaw_rate,
                "safe_yaw_rate": safe.yaw_rate,
            }
        )
    write_csv(out / "cbf_safety.csv", rows)
    write_summary(out / "summary.json", {"samples": len(rows), "min_safe_v": min(r["safe_v"] for r in rows)})

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot([r["obstacle_distance"] for r in rows], [r["safe_v"] for r in rows], label="safe v")
    ax.plot([r["obstacle_distance"] for r in rows], [r["raw_v"] for r in rows], label="raw v")
    ax.invert_xaxis()
    ax.set_title("CBF Safety Layer: speed reduced near obstacle")
    ax.grid(True)
    ax.legend()
    fig.savefig(out / "cbf_safety.png", dpi=140)
    if args.show:
        plt.show()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

