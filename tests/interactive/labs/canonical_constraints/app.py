#!/usr/bin/env python3
"""Interactive canonical constraints explorer using shared core APIs only."""
from __future__ import annotations

import math
import sys
import argparse
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src" / "warehouse_visual_localization"))
from warehouse_visual_localization.core.cbf_filter import filter_linear_cbf_command  # noqa: E402
from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, evaluate_corridor  # noqa: E402
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, curvature_speed_limit  # noqa: E402
from warehouse_visual_localization.core.human_ssm import evaluate_human_ssm  # noqa: E402
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, evaluate_rollover  # noqa: E402
from warehouse_visual_localization.core.types import Pose2D  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke", action="store_true")
    options = parser.parse_args()
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider
    fig = plt.figure(figsize=(14, 8)); fig.canvas.manager.set_window_title("Canonical Forklift Safety Lab")
    corridor_ax = fig.add_axes([0.05, 0.48, 0.46, 0.42])
    speed_ax = fig.add_axes([0.05, 0.11, 0.46, 0.25])
    status_ax = fig.add_axes([0.55, 0.48, 0.18, 0.42]); status_ax.axis("off")
    sliders = {}
    specs = [("speed", 0.0, 0.5, 0.25), ("curvature", 0.0, 2.0, 0.2), ("human dist", 0.2, 5.0, 3.0), ("yaw deg", -45.0, 45.0, 0.0), ("CoG h", 0.25, 1.0, 0.5)]
    for index, (name, low, high, value) in enumerate(specs):
        slider_ax = fig.add_axes([0.78, 0.80 - index * 0.085, 0.17, 0.025]); sliders[name] = Slider(slider_ax, name, low, high, valinit=value)
    reset = Button(fig.add_axes([0.78, 0.16, 0.17, 0.045]), "Reset")
    presets = {
        "Normal": [0.25, 0.2, 3.0, 0.0, 0.5],
        "Human crossing": [0.30, 0.2, 0.65, 0.0, 0.5],
        "Tight turn + high load": [0.32, 1.5, 3.0, 0.0, 0.9],
        "Footprint violation": [0.20, 0.2, 3.0, 25.0, 0.5],
    }
    active_scenario = {"name": "Manual"}
    preset_buttons = []
    for index, name in enumerate(presets):
        button = Button(fig.add_axes([0.55 + (index % 2) * 0.20, 0.32 - (index // 2) * 0.06, 0.18, 0.04]), name)
        preset_buttons.append((button, name))

    def draw(_=None):
        speed, curvature, human, yaw_deg, cog_h = [sliders[key].val for key, *_ in specs]
        cap = curvature_speed_limit([curvature], CurvatureSpeedConfig()).speed_limit[0]
        ssm = evaluate_human_ssm(human, speed)
        roll = evaluate_rollover(speed, curvature, 0.0, [(-0.6,-0.5),(0.6,-0.5),(0.6,0.5),(-0.6,0.5)], RolloverConfig(vehicle_cog_height=cog_h, safety_factor=0.5))
        pose = Pose2D(0.0, 0.65, math.radians(yaw_deg))
        corridor = evaluate_corridor([(0.35,.5),(.35,-.5),(-1.1,-.5),(-1.1,.5)], pose, CorridorBoundary((0,1),1), CorridorBoundary((0,1),-1))
        cbf = filter_linear_cbf_command([speed, 0.2], [[1,0],[-1,0]], [min(cap,ssm.allowed_speed),min(cap,ssm.allowed_speed)], [-.32,-.32],[.32,.32])
        final_cap = min(cap, ssm.allowed_speed)
        corridor_ax.clear(); corridor_ax.set_aspect("equal"); corridor_ax.set_xlim(-1.8, 1.2); corridor_ax.set_ylim(-1.35, 1.35); corridor_ax.grid(True, alpha=.3)
        corridor_ax.axhspan(-1, 1, color="tab:green", alpha=.08, label="Safe corridor")
        corridor_ax.axhline(1, color="black", linewidth=2); corridor_ax.axhline(-1, color="black", linewidth=2)
        poly = corridor.transformed_footprint
        color = "tab:green" if corridor.feasible else "tab:red"
        corridor_ax.fill(poly[:,0], poly[:,1], alpha=.35, color=color); corridor_ax.plot(np.r_[poly[:,0],poly[0,0]], np.r_[poly[:,1],poly[0,1]], color="black", linewidth=2)
        corridor_ax.arrow(pose.x, pose.y, .35 * math.cos(pose.yaw), .35 * math.sin(pose.yaw), width=.025, color="tab:blue")
        corridor_ax.set_title("1. Full footprint inside the black corridor boundaries")
        corridor_ax.text(-1.7, 1.12, "Green: safe", color="tab:green"); corridor_ax.text(-1.7, -1.25, "Red: a corner is outside", color="tab:red")
        speed_ax.clear(); speed_ax.set_ylim(0, .55); speed_ax.set_xlim(-.5, 3.5); speed_ax.set_ylabel("m/s"); speed_ax.set_xticks([0, 1, 2, 3], ["MPPI request", "Curve cap", "Human cap", "CBF output"])
        values = [speed, cap, ssm.allowed_speed, abs(cbf.safe_command[0])]
        bars = speed_ax.bar(range(4), values, color=["tab:blue", "tab:orange", "tab:red", "tab:green"])
        for bar, value in zip(bars, values): speed_ax.text(bar.get_x() + bar.get_width()/2, value + .015, f"{value:.2f}", ha="center")
        speed_ax.axhline(final_cap, color="black", linestyle="--", label=f"Final cap={final_cap:.2f}"); speed_ax.legend(loc="upper right")
        speed_ax.set_title("2. Speed chain: CBF uses the most restrictive safe cap")
        status_ax.clear(); status_ax.axis("off")
        corridor_state = "SAFE" if corridor.feasible else "VIOLATION"
        roll_state = "SAFE" if roll.safe else "RISK"
        ssm_state = ssm.safety_state.upper()
        status_ax.text(0, .94, "3. Current safety state", fontsize=13, fontweight="bold")
        status_ax.text(0, .85, f"Scenario: {active_scenario['name']}", fontsize=10)
        status_ax.text(0, .68, f"Corridor: {corridor_state}\nMargin: {corridor.minimum_margin:.2f} m", color=color, fontsize=11)
        status_ax.text(0, .45, f"Human SSM: {ssm_state}\nDistance: {human:.2f} m", color="tab:red" if ssm_state == "STOP" else "black", fontsize=11)
        status_ax.text(0, .22, f"Rollover: {roll_state}\nLimit utilization: {roll.utilization_ratio:.0%}", color="tab:red" if not roll.safe else "black", fontsize=11)
        status_ax.text(0, .05, "Click a scenario button to load\nvalues and redraw all views.", fontsize=10)
        fig.canvas.draw_idle()
    for slider in sliders.values(): slider.on_changed(draw)
    def reset_values(_):
        active_scenario["name"] = "Manual (reset)"
        for slider in sliders.values():
            slider.reset()
        draw()

    def apply_preset(name):
        def callback(_):
            active_scenario["name"] = name
            for slider, value in zip(sliders.values(), presets[name]):
                slider.set_val(value)
            draw()
        return callback

    reset.on_clicked(reset_values)
    for button, name in preset_buttons:
        button.on_clicked(apply_preset(name))
    draw()
    if options.smoke:
        fig.canvas.draw(); plt.close(fig)
    else:
        plt.show()

if __name__ == "__main__": main()
