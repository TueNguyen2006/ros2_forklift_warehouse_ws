"""Facade that combines optional canonical forklift constraint diagnostics."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from warehouse_visual_localization.core.cbf_filter import CBFProjectionResult, filter_linear_cbf_command
from warehouse_visual_localization.core.corridor_geometry import CorridorBoundary, CorridorResult, evaluate_corridor
from warehouse_visual_localization.core.curvature_speed import CurvatureSpeedConfig, CurvatureSpeedResult, curvature_speed_limit, signed_curvature
from warehouse_visual_localization.core.docking_geometry import DockingConfig, DockingResult, evaluate_docking
from warehouse_visual_localization.core.human_ssm import HumanSSMConfig, HumanSSMResult, evaluate_human_ssm
from warehouse_visual_localization.core.rollover_safety import RolloverConfig, RolloverResult, evaluate_rollover
from warehouse_visual_localization.core.types import Pose2D
from warehouse_visual_localization.core.visual_servo import IBVSResult, PBVSResult, ibvs_command, planar_pbvs_command


@dataclass(frozen=True)
class CanonicalConstraintConfig:
    """Feature gates and conservative demo defaults loaded from separate YAML."""

    enabled: bool = False
    curvature_speed_enabled: bool = True
    footprint_enabled: bool = True
    rollover_enabled: bool = True
    human_ssm_enabled: bool = False
    docking_enabled: bool = False
    visual_servo_enabled: bool = False
    cbf_enabled: bool = False


@dataclass(frozen=True)
class CanonicalConstraintReport:
    curvature_speed: Any = "unavailable"
    corridor: Any = "unavailable"
    rollover: Any = "unavailable"
    human_ssm: Any = "unavailable"
    docking: Any = "unavailable"
    visual_servo: Any = "unavailable"
    cbf: Any = "unavailable"
    overall_safe: bool = True

    def as_dict(self) -> dict[str, Any]:
        def convert(value: Any) -> Any:
            if isinstance(value, np.ndarray):
                return value.tolist()
            if is_dataclass(value):
                return {key: convert(item) for key, item in asdict(value).items()}
            if isinstance(value, dict):
                return {str(key): convert(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [convert(item) for item in value]
            if isinstance(value, np.generic):
                return value.item()
            return value
        return convert(self.__dict__)


class CanonicalConstraintEvaluator:
    """Evaluate independent planning/control/manipulation safety interfaces.

    The evaluator is diagnostic-only. Missing context remains ``unavailable``;
    it never fabricates sensor data, publishes a command, or changes Nav2.
    """

    def __init__(
        self,
        config: CanonicalConstraintConfig | None = None,
        curvature_speed_config: CurvatureSpeedConfig | None = None,
        rollover_config: RolloverConfig | None = None,
        human_ssm_config: HumanSSMConfig | None = None,
        docking_config: DockingConfig | None = None,
    ) -> None:
        self.config = config or CanonicalConstraintConfig()
        self.curvature_speed_config = curvature_speed_config or CurvatureSpeedConfig()
        self.rollover_config = rollover_config or RolloverConfig()
        self.human_ssm_config = human_ssm_config or HumanSSMConfig()
        self.docking_config = docking_config or DockingConfig()

    def evaluate(self, context: Mapping[str, Any]) -> CanonicalConstraintReport:
        if not self.config.enabled:
            return CanonicalConstraintReport()
        outputs: dict[str, Any] = {}
        safe_flags: list[bool] = []

        if self.config.curvature_speed_enabled and "path" in context:
            curvature = signed_curvature(context["path"], int(context.get("smoothing_window", 0)))
            roll_limit = context.get("rollover_lateral_acceleration_limit")
            outputs["curvature_speed"] = curvature_speed_limit(curvature, self.curvature_speed_config, roll_limit)
        else:
            outputs["curvature_speed"] = "unavailable"

        if self.config.footprint_enabled and all(key in context for key in ("body_points", "pose", "left_boundary", "right_boundary")):
            result = evaluate_corridor(context["body_points"], context["pose"], context["left_boundary"], context["right_boundary"])
            outputs["corridor"] = result
            safe_flags.append(result.feasible)
        else:
            outputs["corridor"] = "unavailable"

        rollover_keys = ("speed", "curvature", "longitudinal_acceleration", "support_polygon")
        if self.config.rollover_enabled and all(key in context for key in rollover_keys):
            result = evaluate_rollover(
                context["speed"], context["curvature"], context["longitudinal_acceleration"], context["support_polygon"], self.rollover_config,
                context.get("load_mass", 0.0), context.get("load_cog", (0.0, 0.0)), context.get("combined_cog_height"), context.get("floor_grade", 0.0),
            )
            outputs["rollover"] = result
            safe_flags.append(result.safe)
        else:
            outputs["rollover"] = "unavailable"

        if self.config.human_ssm_enabled and "human_separation" in context and "speed" in context:
            result = evaluate_human_ssm(context["human_separation"], context["speed"], self.human_ssm_config, context.get("human_approach_speed", 0.0))
            outputs["human_ssm"] = result
            safe_flags.append(result.safety_state in ("clear", "speed_limited"))
        else:
            outputs["human_ssm"] = "unavailable"

        if self.config.docking_enabled and "docking_error_y" in context and "docking_error_yaw" in context:
            result = evaluate_docking(context["docking_error_y"], context["docking_error_yaw"], self.docking_config)
            outputs["docking"] = result
            safe_flags.append(result.docking_feasible)
        else:
            outputs["docking"] = "unavailable"

        if self.config.visual_servo_enabled and "ibvs" in context:
            ibvs = context["ibvs"]
            outputs["visual_servo"] = ibvs_command(ibvs["current"], ibvs["desired"], ibvs["interaction_matrix"], ibvs.get("gain", 1.0), ibvs.get("tolerance", 1.0e-6), ibvs.get("limits"))
        elif self.config.visual_servo_enabled and "pbvs" in context:
            pbvs = context["pbvs"]
            outputs["visual_servo"] = planar_pbvs_command(pbvs["current"], pbvs["desired"], pbvs.get("gains", (1.0, 1.0, 1.0)), pbvs.get("limits"))
        else:
            outputs["visual_servo"] = "unavailable"

        if self.config.cbf_enabled and "cbf" in context:
            cbf = context["cbf"]
            result = filter_linear_cbf_command(cbf["desired"], cbf["a_matrix"], cbf["b_vector"], cbf["lower_bounds"], cbf["upper_bounds"], cbf.get("tolerance", 1.0e-8), cbf.get("maximum_iterations", 32))
            outputs["cbf"] = result
            safe_flags.append(result.feasible)
        else:
            outputs["cbf"] = "unavailable"
        return CanonicalConstraintReport(**outputs, overall_safe=all(safe_flags) if safe_flags else True)

