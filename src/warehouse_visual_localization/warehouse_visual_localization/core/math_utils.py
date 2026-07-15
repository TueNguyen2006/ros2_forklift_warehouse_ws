import math


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_abs(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return clamp(value, -limit, limit)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)

