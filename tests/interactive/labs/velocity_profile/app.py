#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import parser, setup_figure, show_or_smoke, slider
from warehouse_visual_localization.core import VelocityProfileConfig, trapezoid_profile


def main() -> None:
    args = parser("Interactive velocity/acceleration profile lab").parse_args()
    fig, ax = setup_figure("Velocity Profile Lab")
    dist = slider(fig, 0, "distance", 1.0, 20.0, 8.0)
    vmax = slider(fig, 1, "v max", 0.05, 1.0, 0.35)
    accel = slider(fig, 2, "accel", 0.05, 1.5, 0.55)
    decel = slider(fig, 3, "decel", 0.05, 1.5, 0.65)

    def redraw() -> None:
        rows = trapezoid_profile(dist.val, 0.05, VelocityProfileConfig(vmax.val, accel.val, decel.val))
        t = [r[0] for r in rows]
        x = [r[1] for r in rows]
        v = [r[2] for r in rows]
        ax.clear()
        ax.grid(True)
        ax.plot(t, v, label="velocity")
        ax.plot(t, x, label="position")
        ax.set_xlabel("time (s)")
        ax.set_title(f"T={t[-1]:.2f}s | distance={x[-1]:.2f}m | max_v={max(v):.2f}m/s")
        ax.legend()
        fig.canvas.draw_idle()

    for widget in [dist, vmax, accel, decel]:
        widget.on_changed(lambda _v: redraw())
    redraw()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
