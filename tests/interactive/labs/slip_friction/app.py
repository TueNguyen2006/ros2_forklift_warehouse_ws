#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import parser, setup_figure, show_or_smoke, slider
from warehouse_visual_localization.core import FrictionConfig, estimate_slip


def main() -> None:
    args = parser("Interactive slip/friction lab").parse_args()
    fig, ax = setup_figure("Slip/Friction Lab")
    req = slider(fig, 0, "force N", -7000, 7000, 2500)
    normal = slider(fig, 1, "normal N", 1000, 10000, 6800)
    wheel_v = slider(fig, 2, "wheel v", -1.0, 1.0, 0.45)
    body_v = slider(fig, 3, "body v", -1.0, 1.0, 0.30)
    mu = slider(fig, 4, "mu static", 0.1, 1.2, 0.75)

    def redraw() -> None:
        slip = estimate_slip(req.val, normal.val, wheel_v.val, body_v.val, FrictionConfig(mu_static=mu.val))
        ax.clear()
        ax.grid(True)
        ax.set_xlim(-8000, 8000)
        ax.set_ylim(0, 3)
        ax.barh([2.0], [req.val], height=0.35, color="orange", label="requested")
        ax.barh([1.4], [slip.traction_force], height=0.35, color="green", label="traction")
        ax.axvline(slip.available_force, color="red", linestyle="--", label="friction limit")
        ax.axvline(-slip.available_force, color="red", linestyle="--")
        ax.set_yticks([1.4, 2.0], ["traction", "requested"])
        ax.set_title(f"slip={slip.slip_ratio:.2f} | saturated={slip.saturated} | available={slip.available_force:.0f} N")
        ax.legend()
        fig.canvas.draw_idle()

    for widget in [req, normal, wheel_v, body_v, mu]:
        widget.on_changed(lambda _v: redraw())
    redraw()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
