#!/usr/bin/env python3
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

try:
    from .scenarios import SCENARIO_BUILDERS
except ImportError:
    from scenarios import SCENARIO_BUILDERS
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


def tracking_error(state: VehicleState, path: list[Pose2D]) -> float:
    return min(math.hypot(state.pose.x - pose.x, state.pose.y - pose.y) for pose in path)


def make_controllers():
    return {
        "Pure Pursuit": PurePursuitController(PurePursuitConfig()),
        "Stanley": StanleyController(StanleyConfig()),
        "MPPI": MPPIController(MPPIConfig()),
        "SBMPC-JAX": SBMPCJAXController(SBMPCJAXConfig()),
    }


def run_case(controller_name: str, scenario_name: str, steps: int = 420) -> dict[str, float | str]:
    model = FourWheelForkliftModel()
    scenario = SCENARIO_BUILDERS[scenario_name]()
    controller = make_controllers()[controller_name]
    state = VehicleState(Pose2D(scenario.initial_pose.x, scenario.initial_pose.y, scenario.initial_pose.yaw))
    errors: list[float] = []
    speeds: list[float] = []
    t0 = time.perf_counter()
    for _ in range(steps):
        command = controller.compute(state, scenario.path)
        state = model.step(state, command, 0.05)
        errors.append(tracking_error(state, scenario.path))
        speeds.append(abs(state.v))
    elapsed = time.perf_counter() - t0
    tail = errors[len(errors) // 2 :]
    return {
        "scenario": scenario_name,
        "controller": controller_name,
        "backend": getattr(controller, "backend", "na"),
        "rms": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "tail_rms": math.sqrt(sum(error * error for error in tail) / len(tail)),
        "max": max(errors),
        "final": errors[-1],
        "mean_v": sum(speeds) / len(speeds),
        "ms_per_step": 1000.0 * elapsed / steps,
    }


def main() -> None:
    print("scenario,controller,backend,rms,tail_rms,max,final,mean_v,ms_per_step")
    for scenario_name in SCENARIO_BUILDERS:
        for controller_name in make_controllers():
            result = run_case(controller_name, scenario_name)
            print(
                f"{result['scenario']},{result['controller']},{result['backend']},"
                f"{result['rms']:.4f},{result['tail_rms']:.4f},{result['max']:.4f},"
                f"{result['final']:.4f},{result['mean_v']:.4f},{result['ms_per_step']:.4f}"
            )


if __name__ == "__main__":
    main()
