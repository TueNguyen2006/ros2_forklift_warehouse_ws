from dataclasses import dataclass

from warehouse_visual_localization.core.math_utils import clamp


@dataclass
class FrictionConfig:
    mu_static: float = 0.75
    mu_dynamic: float = 0.55
    slip_velocity_scale: float = 0.25


@dataclass
class SlipState:
    slip_ratio: float
    traction_force: float
    saturated: bool
    available_force: float


def estimate_slip(
    requested_longitudinal_force: float,
    normal_force: float,
    wheel_speed: float,
    vehicle_speed: float,
    config: FrictionConfig | None = None,
) -> SlipState:
    cfg = config or FrictionConfig()
    speed_ref = max(abs(wheel_speed), abs(vehicle_speed), 1e-3)
    slip_ratio = clamp((wheel_speed - vehicle_speed) / speed_ref, -1.0, 1.0)
    mu = cfg.mu_static - (cfg.mu_static - cfg.mu_dynamic) * min(abs(slip_ratio) / max(cfg.slip_velocity_scale, 1e-3), 1.0)
    available = max(0.0, mu * normal_force)
    traction = clamp(requested_longitudinal_force, -available, available)
    return SlipState(
        slip_ratio=slip_ratio,
        traction_force=traction,
        saturated=abs(requested_longitudinal_force) > available,
        available_force=available,
    )

