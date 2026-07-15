#!/usr/bin/env python3
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common import draw_forklift, parser, setup_figure, show_or_smoke
from warehouse_visual_localization.core import (
    FourWheelForkliftModel,
    MPPIConfig,
    MPPIController,
    Pose2D,
    PurePursuitConfig,
    PurePursuitController,
    SBMPCJAXConfig,
    SBMPCJAXController,
    StanleyConfig,
    StanleyController,
    VehicleState,
)
try:
    from .scenarios import SCENARIO_BUILDERS
except ImportError:
    from scenarios import SCENARIO_BUILDERS


def main() -> None:
    args = parser("Interactive controller tracking lab").parse_args()
    model = FourWheelForkliftModel()
    default_scenario = SCENARIO_BUILDERS["Circle On Path"]()
    state = VehicleState(Pose2D(default_scenario.initial_pose.x, default_scenario.initial_pose.y, default_scenario.initial_pose.yaw))
    trail: list[tuple[float, float]] = []
    last_cmd = {"v": 0.0, "steer": 0.0, "yaw_rate": 0.0}
    playing = {"enabled": False}

    fig, ax = setup_figure("Controller Tracking Lab")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider

    panel_x = 0.735
    panel_w = 0.215
    controller_options = ["Pure Pursuit", "Stanley", "MPPI", "SBMPC-JAX"]
    scenario_options = list(SCENARIO_BUILDERS.keys())
    selected = {"mode": "MPPI", "scenario": "Circle On Path"}
    controller_dropdown = {"open": False}
    scenario_dropdown = {"open": False}

    def section(text: str, y: float):
        title = fig.text(panel_x, y, text, fontsize=10, fontweight="bold", color="#222222")
        line = fig.add_axes([panel_x, y - 0.012, panel_w, 0.002])
        line.set_facecolor("#333333")
        line.set_xticks([])
        line.set_yticks([])
        for spine in line.spines.values():
            spine.set_visible(False)
        return title, line

    def make_slider(y: float, label: str, vmin: float, vmax: float, value: float) -> Slider:
        slider_ax = fig.add_axes([panel_x, y, panel_w, 0.026])
        return Slider(slider_ax, label, vmin, vmax, valinit=value)

    def make_button(x: float, y: float, w: float, label: str) -> Button:
        button_ax = fig.add_axes([x, y, w, 0.038])
        widget = Button(button_ax, label)
        button_ax.set_facecolor("#f2f2f2")
        button_ax.set_zorder(20)
        return widget

    section("Controller", 0.93)
    dropdown_btn = make_button(panel_x, 0.885, panel_w, f"Controller: {selected['mode']}  v")
    option_buttons = [
        make_button(panel_x, 0.842 - i * 0.041, panel_w, option)
        for i, option in enumerate(controller_options)
    ]

    section("Scenario", 0.700)
    scenario_btn = make_button(panel_x, 0.655, panel_w, f"Scenario: {selected['scenario']}  v")
    scenario_buttons = [
        make_button(panel_x, 0.612 - i * 0.041, panel_w, option)
        for i, option in enumerate(scenario_options)
    ]

    common_title, common_line = section("Common", 0.560)
    target_v = make_slider(0.510, "target v", 0.05, 0.35, 0.24)
    amp = make_slider(0.455, "path amp", 0.0, 1.5, 0.35)
    substeps = make_slider(0.400, "substeps", 1, 30, 8)

    algorithm_title = fig.text(panel_x, 0.335, "Algorithm Parameters", fontsize=10, fontweight="bold", color="#222222")
    algorithm_line = fig.add_axes([panel_x, 0.323, panel_w, 0.002])
    algorithm_line.set_facecolor("#333333")
    algorithm_line.set_xticks([])
    algorithm_line.set_yticks([])
    for spine in algorithm_line.spines.values():
        spine.set_visible(False)

    lookahead = make_slider(0.270, "lookahead", 0.25, 2.0, 0.55)
    stanley_gain = make_slider(0.270, "stanley k", 0.1, 4.0, 1.2)
    stanley_heading = make_slider(0.215, "heading gain", 0.1, 3.0, 1.0)
    horizon = make_slider(0.270, "horizon", 6, 50, 36)
    batch_size = make_slider(0.215, "batch size", 16, 400, 320)
    temperature = make_slider(0.160, "temperature", 0.05, 2.0, 0.60)
    lambda_mpc = make_slider(0.160, "lambda", 0.1, 12.0, 5.0)

    actions_title, actions_line = section("Actions", 0.135)
    play_btn = make_button(panel_x, 0.080, 0.095, "Play")
    step_btn = make_button(panel_x + 0.115, 0.080, 0.095, "Step")
    reset_btn = make_button(panel_x, 0.025, panel_w, "Reset")

    lower_panel_artists = [
        target_v.ax,
        amp.ax,
        substeps.ax,
        common_title,
        common_line,
        algorithm_title,
        algorithm_line,
        lookahead.ax,
        stanley_gain.ax,
        stanley_heading.ax,
        horizon.ax,
        batch_size.ax,
        temperature.ax,
        lambda_mpc.ax,
        play_btn.ax,
        step_btn.ax,
        reset_btn.ax,
        actions_title,
        actions_line,
    ]

    mppi = MPPIController()
    sbmpc = SBMPCJAXController()
    stanley = StanleyController()

    def selected_mode() -> str:
        return selected["mode"]

    def current_scenario():
        return SCENARIO_BUILDERS[selected["scenario"]](amp.val)

    def reset_state_for_scenario() -> None:
        nonlocal state, trail, mppi, sbmpc, stanley
        scenario = current_scenario()
        state = VehicleState(Pose2D(scenario.initial_pose.x, scenario.initial_pose.y, scenario.initial_pose.yaw))
        trail = []
        mppi = MPPIController()
        sbmpc = SBMPCJAXController()
        stanley = StanleyController()
        playing["enabled"] = False
        play_btn.label.set_text("Play")

    def controller_label() -> str:
        if selected_mode() == "MPPI":
            return f"MPPI/{mppi.backend}"
        if selected_mode() == "SBMPC-JAX":
            return f"SBMPC/{sbmpc.backend}"
        return selected_mode()

    def sync_parameter_visibility() -> None:
        mode_name = selected_mode()
        scenario_menu_open = scenario_dropdown["open"]
        for widget in [
            lookahead,
            stanley_gain,
            stanley_heading,
            horizon,
            batch_size,
            temperature,
            lambda_mpc,
        ]:
            widget.ax.set_visible(False)
        if not scenario_menu_open:
            if mode_name == "Pure Pursuit":
                lookahead.ax.set_visible(True)
            elif mode_name == "Stanley":
                stanley_gain.ax.set_visible(True)
                stanley_heading.ax.set_visible(True)
            elif mode_name == "MPPI":
                horizon.ax.set_visible(True)
                batch_size.ax.set_visible(True)
                temperature.ax.set_visible(True)
            else:
                horizon.ax.set_visible(True)
                batch_size.ax.set_visible(True)
                lambda_mpc.ax.set_visible(True)
        for option_button in option_buttons:
            option_button.ax.set_visible(controller_dropdown["open"])
            option_button.ax.set_zorder(30)
        for scenario_button in scenario_buttons:
            scenario_button.ax.set_visible(scenario_dropdown["open"])
            scenario_button.ax.set_zorder(40)
        for artist in lower_panel_artists:
            if artist in [lookahead.ax, stanley_gain.ax, stanley_heading.ax, horizon.ax, batch_size.ax, temperature.ax, lambda_mpc.ax]:
                continue
            artist.set_visible(not scenario_menu_open)
        dropdown_btn.label.set_text(f"Controller: {selected['mode']}  v")
        scenario_btn.label.set_text(f"Scenario: {selected['scenario']}  v")
        fig.canvas.draw_idle()

    def compute_command(path):
        if selected_mode() == "Pure Pursuit":
            ctrl = PurePursuitController(
                PurePursuitConfig(lookahead=lookahead.val, target_speed=target_v.val)
            )
            cmd = ctrl.compute(state, path)
        elif selected_mode() == "Stanley":
            stanley.config = StanleyConfig(
                target_speed=target_v.val,
                cross_track_gain=stanley_gain.val,
                heading_gain=stanley_heading.val,
            )
            cmd = stanley.compute(state, path)
        elif selected_mode() == "MPPI":
            mppi.config = MPPIConfig(
                target_speed=target_v.val,
                horizon_steps=int(horizon.val),
                batch_size=int(batch_size.val),
                temperature=temperature.val,
            )
            cmd = mppi.compute(state, path)
        else:
            sbmpc.config = SBMPCJAXConfig(
                target_speed=target_v.val,
                horizon_steps=int(horizon.val),
                batch_size=int(batch_size.val),
                lambda_mpc=lambda_mpc.val,
            )
            cmd = sbmpc.compute(state, path)
        last_cmd["v"] = cmd.v
        last_cmd["steer"] = cmd.steer
        last_cmd["yaw_rate"] = cmd.yaw_rate
        return controller_label(), cmd

    def redraw() -> None:
        scenario = current_scenario()
        path = scenario.path
        mode_name = controller_label()
        ax.clear()
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.set_xlim(*scenario.xlim)
        ax.set_ylim(*scenario.ylim)
        ax.plot([p.x for p in path], [p.y for p in path], "k--", label="reference")
        if trail:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], "r-", label="tracked")
        draw_forklift(ax, state)
        err = min(math.hypot(state.pose.x - p.x, state.pose.y - p.y) for p in path)
        status = "PLAY" if playing["enabled"] else "PAUSE"
        max_curvature = scenario.max_curvature
        required_steer = math.atan(max_curvature * model.params.wheelbase)
        feasible = abs(required_steer) <= model.params.max_steer
        feasibility = "feasible" if feasible else f"infeasible steer>{model.params.max_steer:.2f}"
        ax.set_title(
            f"{mode_name} | {status} | tracking_error={err:.3f} m | target_v={target_v.val:.2f}\n"
            f"cmd_v={last_cmd['v']:.2f} steer={last_cmd['steer']:.2f} yaw_rate={last_cmd['yaw_rate']:.2f} | "
            f"{scenario.name} steer_req~{required_steer:.2f} rad: {feasibility}"
        )
        ax.legend()
        fig.canvas.draw_idle()

    def step_once() -> None:
        nonlocal state
        path = current_scenario().path
        _, cmd = compute_command(path)
        state = model.step(state, cmd, 0.05)
        trail.append((state.pose.x, state.pose.y))

    def step(_event=None) -> None:
        for _ in range(int(substeps.val)):
            step_once()
        redraw()

    def reset(_event=None) -> None:
        reset_state_for_scenario()
        redraw()

    def toggle_play(_event=None) -> None:
        playing["enabled"] = not playing["enabled"]
        play_btn.label.set_text("Pause" if playing["enabled"] else "Play")
        redraw()

    def on_timer() -> bool:
        if playing["enabled"]:
            step()
        return True

    timer = fig.canvas.new_timer(interval=50)
    timer.add_callback(on_timer)
    timer.start()

    play_btn.on_clicked(toggle_play)
    step_btn.on_clicked(step)
    reset_btn.on_clicked(reset)

    def toggle_dropdown(_event=None) -> None:
        controller_dropdown["open"] = not controller_dropdown["open"]
        scenario_dropdown["open"] = False
        sync_parameter_visibility()

    def toggle_scenario_dropdown(_event=None) -> None:
        scenario_dropdown["open"] = not scenario_dropdown["open"]
        controller_dropdown["open"] = False
        sync_parameter_visibility()

    def choose_mode(mode_name: str):
        def _handler(_event=None) -> None:
            selected["mode"] = mode_name
            controller_dropdown["open"] = False
            sync_parameter_visibility()
            redraw()
        return _handler

    def choose_scenario(scenario_name: str):
        def _handler(_event=None) -> None:
            selected["scenario"] = scenario_name
            scenario_dropdown["open"] = False
            reset_state_for_scenario()
            sync_parameter_visibility()
            redraw()
        return _handler

    dropdown_btn.on_clicked(toggle_dropdown)
    scenario_btn.on_clicked(toggle_scenario_dropdown)
    for option_button, option_name in zip(option_buttons, controller_options):
        option_button.on_clicked(choose_mode(option_name))
    for scenario_button, scenario_name in zip(scenario_buttons, scenario_options):
        scenario_button.on_clicked(choose_scenario(scenario_name))

    for widget in [
        lookahead,
        stanley_gain,
        stanley_heading,
        target_v,
        amp,
        horizon,
        batch_size,
        temperature,
        lambda_mpc,
        substeps,
    ]:
        widget.on_changed(lambda _v: redraw())
    sync_parameter_visibility()
    redraw()
    if args.smoke:
        step()
    show_or_smoke(fig, args.smoke)


if __name__ == "__main__":
    main()
