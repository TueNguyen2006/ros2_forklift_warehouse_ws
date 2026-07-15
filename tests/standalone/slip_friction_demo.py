#!/usr/bin/env python3

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import estimate_slip


def main() -> None:
    args = parser("Slip and friction standalone demo").parse_args()
    vehicle, _, _, friction_cfg, _, _ = load_vehicle_config(args.config)
    out = ensure_output_dir(args.output_dir, "slip_friction")
    normal_force = vehicle.mass * vehicle.gravity * 0.5
    rows = []
    for i in range(101):
        requested = -6000.0 + i * 120.0
        wheel_speed = 0.30 + 0.006 * i
        vehicle_speed = 0.30
        slip = estimate_slip(requested, normal_force, wheel_speed, vehicle_speed, friction_cfg)
        rows.append(
            {
                "requested_force": requested,
                "wheel_speed": wheel_speed,
                "vehicle_speed": vehicle_speed,
                "slip_ratio": slip.slip_ratio,
                "traction_force": slip.traction_force,
                "available_force": slip.available_force,
                "saturated": int(slip.saturated),
            }
        )
    write_csv(out / "slip_friction.csv", rows)
    write_summary(out / "summary.json", {"saturated_count": sum(r["saturated"] for r in rows), "samples": len(rows)})

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot([r["requested_force"] for r in rows], [r["traction_force"] for r in rows], label="traction")
    ax.plot([r["requested_force"] for r in rows], [r["available_force"] for r in rows], label="available +")
    ax.plot([r["requested_force"] for r in rows], [-r["available_force"] for r in rows], label="available -")
    ax.set_title("Slip/Friction Force Saturation")
    ax.grid(True)
    ax.legend()
    fig.savefig(out / "slip_friction.png", dpi=140)
    if args.show:
        plt.show()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

