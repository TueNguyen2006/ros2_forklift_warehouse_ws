import math
from dataclasses import dataclass

from warehouse_visual_localization.core import Pose2D


@dataclass(frozen=True)
class ControllerScenario:
    name: str
    path: list[Pose2D]
    initial_pose: Pose2D
    max_curvature: float
    xlim: tuple[float, float]
    ylim: tuple[float, float]


def _path_from_xy(points: list[tuple[float, float]]) -> list[Pose2D]:
    path: list[Pose2D] = []
    for i, (x, y) in enumerate(points):
        if i == 0:
            nx, ny = points[1]
            yaw = math.atan2(ny - y, nx - x)
        elif i == len(points) - 1:
            px, py = points[i - 1]
            yaw = math.atan2(y - py, x - px)
        else:
            px, py = points[i - 1]
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - py, nx - px)
        path.append(Pose2D(x, y, yaw))
    return path


def _max_curvature(path: list[Pose2D]) -> float:
    max_curv = 0.0
    for i in range(1, len(path) - 1):
        p0 = path[i - 1]
        p1 = path[i]
        p2 = path[i + 1]
        ds = math.hypot(p2.x - p0.x, p2.y - p0.y)
        if ds <= 1e-6:
            continue
        dyaw = math.atan2(math.sin(p2.yaw - p0.yaw), math.cos(p2.yaw - p0.yaw))
        max_curv = max(max_curv, abs(dyaw / ds))
    return max_curv


def sine_offset(amplitude: float = 0.35, length: float = 7.0) -> ControllerScenario:
    path = []
    for i in range(180):
        x = length * i / 179.0
        y = amplitude * math.sin(x)
        yaw = math.atan2(amplitude * math.cos(x), 1.0)
        path.append(Pose2D(x, y, yaw))
    return ControllerScenario(
        name="Sine Offset",
        path=path,
        initial_pose=Pose2D(0.0, -0.25, 0.0),
        max_curvature=abs(amplitude),
        xlim=(-1.0, 8.0),
        ylim=(-3.0, 3.0),
    )


def circle_on_path(_amplitude: float = 0.35, radius: float = 2.2) -> ControllerScenario:
    center_x = 2.0
    center_y = 0.0
    path = []
    for i in range(260):
        theta = math.pi + 2.0 * math.pi * i / 259.0
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        yaw = math.atan2(math.cos(theta), -math.sin(theta))
        path.append(Pose2D(x, y, yaw))
    return ControllerScenario(
        name="Circle On Path",
        path=path,
        initial_pose=path[0],
        max_curvature=1.0 / radius,
        xlim=(-1.0, 5.0),
        ylim=(-3.2, 3.2),
    )


def u_turn_start_line(_amplitude: float = 0.35, radius: float = 2.0, straight: float = 2.5) -> ControllerScenario:
    path: list[Pose2D] = []
    for i in range(70):
        x = straight * i / 69.0
        path.append(Pose2D(x, 0.0, 0.0))
    for i in range(130):
        theta = -math.pi / 2.0 + math.pi * i / 129.0
        x = straight + radius * math.cos(theta)
        y = radius + radius * math.sin(theta)
        yaw = theta + math.pi / 2.0
        path.append(Pose2D(x, y, yaw))
    for i in range(70):
        x = straight * (1.0 - i / 69.0)
        path.append(Pose2D(x, 2.0 * radius, math.pi))
    return ControllerScenario(
        name="U-Turn Start Line",
        path=path,
        initial_pose=path[0],
        max_curvature=1.0 / radius,
        xlim=(-1.0, straight + radius + 1.0),
        ylim=(-1.0, 2.0 * radius + 1.0),
    )


def figure_eight_on_path(_amplitude: float = 0.35, scale: float = 9.5) -> ControllerScenario:
    points = []
    for i in range(360):
        t = 2.0 * math.pi * i / 359.0
        x = scale * math.sin(t)
        y = 0.55 * scale * math.sin(2.0 * t)
        points.append((x, y))
    path = _path_from_xy(points)
    return ControllerScenario(
        name="Figure Eight On Path",
        path=path,
        initial_pose=path[0],
        max_curvature=_max_curvature(path),
        xlim=(-10.5, 10.5),
        ylim=(-6.0, 6.0),
    )


def warehouse_chicane(_amplitude: float = 0.35, length: float = 8.0) -> ControllerScenario:
    points = []
    for i in range(260):
        x = length * i / 259.0
        y = 0.35 * math.sin(2.0 * math.pi * x / length) + 0.16 * math.sin(4.0 * math.pi * x / length)
        points.append((x, y))
    path = _path_from_xy(points)
    return ControllerScenario(
        name="Warehouse Chicane",
        path=path,
        initial_pose=path[0],
        max_curvature=_max_curvature(path),
        xlim=(-1.0, length + 1.0),
        ylim=(-2.2, 2.2),
    )


def tight_hairpin_stress(_amplitude: float = 0.35, radius: float = 1.25, straight: float = 2.0) -> ControllerScenario:
    path: list[Pose2D] = []
    for i in range(55):
        x = straight * i / 54.0
        path.append(Pose2D(x, 0.0, 0.0))
    for i in range(120):
        theta = -math.pi / 2.0 + math.pi * i / 119.0
        x = straight + radius * math.cos(theta)
        y = radius + radius * math.sin(theta)
        yaw = theta + math.pi / 2.0
        path.append(Pose2D(x, y, yaw))
    for i in range(55):
        x = straight * (1.0 - i / 54.0)
        path.append(Pose2D(x, 2.0 * radius, math.pi))
    return ControllerScenario(
        name="Tight Hairpin Stress",
        path=path,
        initial_pose=path[0],
        max_curvature=1.0 / radius,
        xlim=(-1.0, straight + radius + 1.0),
        ylim=(-1.0, 2.0 * radius + 1.0),
    )


SCENARIO_BUILDERS = {
    "Sine Offset": sine_offset,
    "Circle On Path": circle_on_path,
    "U-Turn Start Line": u_turn_start_line,
    "Figure Eight On Path": figure_eight_on_path,
    "Warehouse Chicane": warehouse_chicane,
    "Tight Hairpin Stress": tight_hairpin_stress,
}
