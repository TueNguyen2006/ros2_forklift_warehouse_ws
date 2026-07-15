from dataclasses import dataclass


@dataclass
class VelocityProfileConfig:
    max_speed: float = 0.35
    max_accel: float = 0.55
    max_decel: float = 0.65


def trapezoid_profile(distance: float, dt: float, config: VelocityProfileConfig | None = None) -> list[tuple[float, float, float]]:
    cfg = config or VelocityProfileConfig()
    samples: list[tuple[float, float, float]] = []
    t = 0.0
    x = 0.0
    v = 0.0
    total = max(distance, 0.0)
    while x < total - 1e-6 and t < 120.0:
        braking_distance = v * v / max(2.0 * cfg.max_decel, 1e-6)
        if total - x <= braking_distance:
            a = -cfg.max_decel
        elif v < cfg.max_speed:
            a = cfg.max_accel
        else:
            a = 0.0
        v = max(0.0, min(cfg.max_speed, v + a * dt))
        x = min(total, x + v * dt)
        samples.append((t, x, v))
        t += dt
    return samples

