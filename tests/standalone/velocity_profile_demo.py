#!/usr/bin/env python3

from common import ensure_output_dir, load_vehicle_config, parser, write_csv, write_summary
from warehouse_visual_localization.core import trapezoid_profile


def main() -> None:
    args = parser("Velocity and acceleration profile demo").parse_args()
    *_, profile_cfg, _ = load_vehicle_config(args.config)
    out = ensure_output_dir(args.output_dir, "velocity_profile")
    samples = trapezoid_profile(8.0, args.dt, profile_cfg)
    rows = []
    prev_v = 0.0
    for t, x, v in samples:
        rows.append({"t": t, "x": x, "v": v, "a": (v - prev_v) / args.dt})
        prev_v = v
    write_csv(out / "velocity_profile.csv", rows)
    write_summary(out / "summary.json", {"samples": len(rows), "final": rows[-1]})

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot([r["t"] for r in rows], [r["v"] for r in rows])
    ax[0].set_ylabel("v m/s")
    ax[0].grid(True)
    ax[1].plot([r["t"] for r in rows], [r["a"] for r in rows])
    ax[1].set_ylabel("a m/s2")
    ax[1].set_xlabel("t s")
    ax[1].grid(True)
    fig.savefig(out / "velocity_profile.png", dpi=140)
    if args.show:
        plt.show()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

