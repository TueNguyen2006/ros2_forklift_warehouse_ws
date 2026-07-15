import math
import os
import importlib.util
from pathlib import Path
from dataclasses import dataclass

from warehouse_visual_localization.core.math_utils import clamp, distance_xy, wrap_angle
from warehouse_visual_localization.core.types import ControlCommand, Pose2D, VehicleParams, VehicleState
from warehouse_visual_localization.core.vehicle_model import FourWheelForkliftModel

try:
    from warehouse_visual_localization import _mppi_native
except Exception:  # pragma: no cover - optional compiled acceleration.
    _mppi_native = None
    for parent in Path(__file__).resolve().parents:
        candidate_root = parent
        candidate = candidate_root / "install" / "warehouse_visual_localization" / "lib" / "python3.10" / "site-packages" / "warehouse_visual_localization"
        matches = sorted(candidate.glob("_mppi_native*.so"))
        if matches:
            spec = importlib.util.spec_from_file_location("warehouse_visual_localization._mppi_native", matches[0])
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                _mppi_native = module
                break

try:
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    import jax
    import jax.numpy as jnp
except Exception:  # pragma: no cover - optional JAX controller backend.
    jax = None
    jnp = None


def _is_closed_path(path: list[Pose2D], tolerance: float = 0.25) -> bool:
    return len(path) > 2 and distance_xy(path[0].x, path[0].y, path[-1].x, path[-1].y) <= tolerance


@dataclass
class PurePursuitConfig:
    lookahead: float = 0.55
    target_speed: float = 0.24
    goal_tolerance: float = 0.18
    yaw_rate_limit: float = 0.45


@dataclass
class MPPIConfig:
    target_speed: float = 0.24
    horizon_steps: int = 36
    dt: float = 0.05
    batch_size: int = 320
    max_steer: float = 0.62
    speed_noise_std: float = 0.05
    steer_noise_std: float = 0.22
    temperature: float = 0.6
    path_weight: float = 25.0
    heading_weight: float = 3.0
    goal_weight: float = 2.0
    control_weight: float = 0.15
    smooth_weight: float = 0.35
    progress_weight: float = 2.0
    speed_weight: float = 40.0
    guidance_blend: float = 0.45
    guidance_lookahead: float = 0.65


@dataclass
class SBMPCJAXConfig:
    target_speed: float = 0.24
    horizon_steps: int = 36
    dt: float = 0.05
    batch_size: int = 320
    max_steer: float = 0.62
    speed_noise_std: float = 0.05
    steer_noise_std: float = 0.24
    lambda_mpc: float = 5.0
    path_weight: float = 25.0
    heading_weight: float = 3.0
    goal_weight: float = 2.0
    control_weight: float = 0.12
    smooth_weight: float = 0.32
    progress_weight: float = 2.0
    speed_weight: float = 40.0
    guidance_blend: float = 0.45
    guidance_lookahead: float = 0.65


class PurePursuitController:
    """Standalone controller used to validate path tracking independent of Nav2."""

    def __init__(self, config: PurePursuitConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        self.config = config or PurePursuitConfig()
        self.vehicle = vehicle or VehicleParams()
        self.model = FourWheelForkliftModel(self.vehicle)

    def _target_index(self, pose: Pose2D, path: list[Pose2D]) -> int:
        nearest = min(range(len(path)), key=lambda i: distance_xy(pose.x, pose.y, path[i].x, path[i].y))
        for i in range(nearest, len(path)):
            if distance_xy(pose.x, pose.y, path[i].x, path[i].y) >= self.config.lookahead:
                return i
        return len(path) - 1

    def compute(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        if not path:
            return ControlCommand()
        goal = path[-1]
        if not _is_closed_path(path) and distance_xy(state.pose.x, state.pose.y, goal.x, goal.y) <= self.config.goal_tolerance:
            return ControlCommand()
        target = path[self._target_index(state.pose, path)]
        heading = math.atan2(target.y - state.pose.y, target.x - state.pose.x)
        alpha = wrap_angle(heading - state.pose.yaw)
        curvature = 2.0 * math.sin(alpha) / max(self.config.lookahead, 1e-3)
        yaw_rate = clamp(
            self.config.target_speed * curvature,
            -abs(self.config.yaw_rate_limit),
            abs(self.config.yaw_rate_limit),
        )
        steer = self.model.steer_from_yaw_rate(self.config.target_speed, yaw_rate)
        return ControlCommand(v=self.config.target_speed, yaw_rate=yaw_rate, steer=steer)


class MPPIController:
    """Standalone MPPI controller for controller-lab inspection.

    This implements the core MPPI loop: sample noisy control sequences around a
    nominal sequence, roll out all trajectories, score them, compute softmin
    weights, update the nominal sequence, and apply the first control.
    """

    def __init__(self, config: MPPIConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        import random

        self.config = config or MPPIConfig()
        self.vehicle = vehicle or VehicleParams()
        self.model = FourWheelForkliftModel(self.vehicle)
        self.rng = random.Random(11)
        self.nominal: list[tuple[float, float]] = [
            (self.config.target_speed, 0.0) for _ in range(self.config.horizon_steps)
        ]
        self.call_count = 0
        self.backend = "cpp" if _mppi_native is not None else "python"

    def compute(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        if not path:
            return ControlCommand()
        if not _is_closed_path(path) and distance_xy(state.pose.x, state.pose.y, path[-1].x, path[-1].y) < 0.18:
            return ControlCommand()

        self._sync_nominal()
        self._apply_guidance(state, path)
        if _mppi_native is not None:
            return self._compute_native(state, path)

        sequences = [self.nominal]
        for _ in range(max(1, self.config.batch_size - 1)):
            sequences.append(self._sample_sequence())

        costs = [self._rollout_cost(state, sequence, path) for sequence in sequences]
        min_cost = min(costs)
        temperature = max(self.config.temperature, 1e-6)
        weights = [math.exp(-(cost - min_cost) / temperature) for cost in costs]
        weight_sum = max(sum(weights), 1e-9)

        updated: list[tuple[float, float]] = []
        for t in range(self.config.horizon_steps):
            v = sum(weights[i] * sequences[i][t][0] for i in range(len(sequences))) / weight_sum
            steer = sum(weights[i] * sequences[i][t][1] for i in range(len(sequences))) / weight_sum
            updated.append((self._clamp_speed(v), self._clamp_steer(steer)))
        self.nominal = updated[1:] + [(self.config.target_speed, 0.0)]

        v0, steer0 = updated[0]
        yaw_rate = self.model.yaw_rate_from_steer(v0, steer0)
        return ControlCommand(v=v0, yaw_rate=yaw_rate, steer=steer0)

    def _compute_native(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        path_x = [p.x for p in path]
        path_y = [p.y for p in path]
        path_yaw = [p.yaw for p in path]
        nominal_v = [v for v, _steer in self.nominal]
        nominal_steer = [steer for _v, steer in self.nominal]
        seed = 1109 + self.call_count * 104729
        self.call_count += 1

        v0, steer0, yaw_rate, updated = _mppi_native.mppi_step(
            state.pose.x,
            state.pose.y,
            state.pose.yaw,
            state.v,
            state.steer,
            path_x,
            path_y,
            path_yaw,
            nominal_v,
            nominal_steer,
            self.config.target_speed,
            int(self.config.horizon_steps),
            int(self.config.batch_size),
            self.config.dt,
            self.config.max_steer,
            self.config.speed_noise_std,
            self.config.steer_noise_std,
            self.config.temperature,
            self.config.path_weight,
            self.config.heading_weight,
            self.config.goal_weight,
            self.config.control_weight,
            self.config.smooth_weight,
            self.config.progress_weight,
            self.config.speed_weight,
            self.vehicle.wheelbase,
            self.vehicle.max_accel,
            self.vehicle.max_decel,
            self.vehicle.max_steer_rate,
            seed,
            0.03,
            abs(self.config.target_speed),
        )
        self.nominal = list(updated)[1:] + [(self.config.target_speed, 0.0)]
        return ControlCommand(v=v0, yaw_rate=yaw_rate, steer=steer0)

    def _sync_nominal(self) -> None:
        horizon = max(2, int(self.config.horizon_steps))
        if len(self.nominal) != horizon:
            self.nominal = [(self.config.target_speed, 0.0) for _ in range(horizon)]
        self.config.horizon_steps = horizon

    def _apply_guidance(self, state: VehicleState, path: list[Pose2D]) -> None:
        guide = self._pure_pursuit_guidance(state, path)
        blend = clamp(self.config.guidance_blend, 0.0, 1.0)
        self.nominal = [
            (
                (1.0 - blend) * v + blend * guide.v,
                self._clamp_steer((1.0 - blend) * steer + blend * guide.steer),
            )
            for v, steer in self.nominal
        ]

    def _pure_pursuit_guidance(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        nearest = min(range(len(path)), key=lambda i: distance_xy(state.pose.x, state.pose.y, path[i].x, path[i].y))
        lookahead = max(self.config.guidance_lookahead, 1e-3)
        target = path[-1]
        for i in range(nearest, len(path)):
            if distance_xy(state.pose.x, state.pose.y, path[i].x, path[i].y) >= lookahead:
                target = path[i]
                break
        heading = math.atan2(target.y - state.pose.y, target.x - state.pose.x)
        alpha = wrap_angle(heading - state.pose.yaw)
        curvature = 2.0 * math.sin(alpha) / lookahead
        yaw_rate = self.config.target_speed * curvature
        steer = self.model.steer_from_yaw_rate(self.config.target_speed, yaw_rate)
        return ControlCommand(v=self.config.target_speed, yaw_rate=yaw_rate, steer=steer)

    def _sample_sequence(self) -> list[tuple[float, float]]:
        sequence = []
        for v_nominal, steer_nominal in self.nominal:
            v = self._clamp_speed(v_nominal + self.rng.gauss(0.0, self.config.speed_noise_std))
            steer = self._clamp_steer(steer_nominal + self.rng.gauss(0.0, self.config.steer_noise_std))
            sequence.append((v, steer))
        return sequence

    def _clamp_speed(self, speed: float) -> float:
        return clamp(speed, 0.03, abs(self.config.target_speed))

    def _clamp_steer(self, steer: float) -> float:
        return clamp(steer, -abs(self.config.max_steer), abs(self.config.max_steer))

    def _rollout_cost(self, state: VehicleState, sequence: list[tuple[float, float]], path: list[Pose2D]) -> float:
        sim = VehicleState(Pose2D(state.pose.x, state.pose.y, state.pose.yaw), v=state.v, yaw_rate=state.yaw_rate, steer=state.steer)
        cost = 0.0
        start_progress = self._nearest_index(sim.pose, path)
        previous_steer = sim.steer
        for k, (speed, steer) in enumerate(sequence):
            command = ControlCommand(v=speed, steer=steer)
            sim = self.model.step(sim, command, self.config.dt)
            nearest = self._nearest_index(sim.pose, path)
            target = path[nearest]
            path_error = distance_xy(sim.pose.x, sim.pose.y, target.x, target.y)
            heading_error = abs(wrap_angle(target.yaw - sim.pose.yaw))
            step_scale = 1.0 + 0.04 * k
            cost += step_scale * (
                self.config.path_weight * path_error * path_error
                + self.config.heading_weight * heading_error * heading_error
                + self.config.control_weight * steer * steer
                + self.config.smooth_weight * (steer - previous_steer) * (steer - previous_steer)
                + self.config.speed_weight * (speed - self.config.target_speed) * (speed - self.config.target_speed)
            )
            previous_steer = steer
        end_progress = self._nearest_index(sim.pose, path)
        goal_error = distance_xy(sim.pose.x, sim.pose.y, path[-1].x, path[-1].y)
        cost += self.config.goal_weight * goal_error
        cost -= self.config.progress_weight * max(0, end_progress - start_progress)
        return cost

    @staticmethod
    def _nearest_index(pose: Pose2D, path: list[Pose2D]) -> int:
        return min(range(len(path)), key=lambda i: distance_xy(pose.x, pose.y, path[i].x, path[i].y))


if jax is not None:

    @jax.jit
    def _sbmpc_wrap_angle(angle):
        return jnp.arctan2(jnp.sin(angle), jnp.cos(angle))

    def _make_sbmpc_step(horizon_steps: int, batch_size: int):
        @jax.jit
        def _step(
            state,
            path_x,
            path_y,
            path_yaw,
            nominal,
            key,
            target_speed,
            dt,
            max_steer,
            speed_noise_std,
            steer_noise_std,
            lambda_mpc,
            path_weight,
            heading_weight,
            goal_weight,
            control_weight,
            smooth_weight,
            progress_weight,
            speed_weight,
            wheelbase,
            max_accel,
            max_decel,
            max_steer_rate,
        ):
            noise = jax.random.normal(key, (batch_size, horizon_steps, 2), dtype=jnp.float32)
            samples = nominal[None, :, :] + noise * jnp.array([speed_noise_std, steer_noise_std], dtype=jnp.float32)
            samples = samples.at[0, :, :].set(nominal)
            samples = samples.at[:, :, 0].set(jnp.clip(samples[:, :, 0], 0.03, jnp.abs(target_speed)))
            samples = samples.at[:, :, 1].set(jnp.clip(samples[:, :, 1], -jnp.abs(max_steer), jnp.abs(max_steer)))

            def rollout(sequence):
                start_d2 = (state[0] - path_x) ** 2 + (state[1] - path_y) ** 2
                start_index = jnp.argmin(start_d2)

                def one_step(carry, item):
                    sim_state, previous_steer, cost = carry
                    idx, control = item
                    cmd_v = control[0]
                    cmd_steer = control[1]

                    steer_delta = jnp.clip(
                        cmd_steer - sim_state[4],
                        -jnp.abs(max_steer_rate) * dt,
                        jnp.abs(max_steer_rate) * dt,
                    )
                    steer = jnp.clip(sim_state[4] + steer_delta, -jnp.abs(max_steer), jnp.abs(max_steer))
                    accel_limit = jnp.where(cmd_v >= sim_state[3], max_accel, max_decel)
                    accel = jnp.clip(cmd_v - sim_state[3], -accel_limit * dt, accel_limit * dt) / jnp.maximum(dt, 1e-6)
                    v = sim_state[3] + accel * dt
                    yaw_rate = jnp.where(
                        (jnp.abs(steer) < 1e-5) | (jnp.abs(v) < 1e-5),
                        0.0,
                        -v * jnp.tan(steer) / jnp.maximum(wheelbase, 1e-6),
                    )
                    yaw = _sbmpc_wrap_angle(sim_state[2] + yaw_rate * dt)
                    x = sim_state[0] + v * jnp.cos(yaw) * dt
                    y = sim_state[1] + v * jnp.sin(yaw) * dt
                    next_state = jnp.array([x, y, yaw, v, steer], dtype=jnp.float32)

                    d2 = (x - path_x) ** 2 + (y - path_y) ** 2
                    nearest = jnp.argmin(d2)
                    path_error = jnp.sqrt(jnp.min(d2))
                    heading_error = jnp.abs(_sbmpc_wrap_angle(path_yaw[nearest] - yaw))
                    step_scale = 1.0 + 0.04 * idx
                    step_cost = step_scale * (
                        path_weight * path_error * path_error
                        + heading_weight * heading_error * heading_error
                        + control_weight * steer * steer
                        + smooth_weight * (steer - previous_steer) * (steer - previous_steer)
                        + speed_weight * (v - target_speed) * (v - target_speed)
                    )
                    return (next_state, steer, cost + step_cost), nearest

                (final_state, _, cost), nearest_history = jax.lax.scan(
                    one_step,
                    (state, state[4], jnp.array(0.0, dtype=jnp.float32)),
                    (jnp.arange(horizon_steps, dtype=jnp.float32), sequence),
                )
                goal_error = jnp.sqrt((final_state[0] - path_x[-1]) ** 2 + (final_state[1] - path_y[-1]) ** 2)
                progress = jnp.maximum(0, nearest_history[-1] - start_index)
                cost = cost + goal_weight * goal_error - progress_weight * progress
                return cost

            costs = jax.vmap(rollout)(samples)
            best_cost = jnp.min(costs)
            weights = jnp.exp(-lambda_mpc * (costs - best_cost))
            weights = weights / jnp.maximum(jnp.sum(weights), 1e-9)
            updated = jnp.sum(weights[:, None, None] * samples, axis=0)
            return updated, costs

        return _step

else:

    def _make_sbmpc_step(_horizon_steps: int, _batch_size: int):
        return None


class SBMPCJAXController:
    """JAX sampling-based MPC inspired by tombelv/sbmpc for the controller lab."""

    def __init__(self, config: SBMPCJAXConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        self.config = config or SBMPCJAXConfig()
        self.vehicle = vehicle or VehicleParams()
        self.model = FourWheelForkliftModel(self.vehicle)
        self.backend = "jax" if jax is not None else "unavailable"
        self.key = jax.random.PRNGKey(420) if jax is not None else None
        self.compiled_signature: tuple[int, int] | None = None
        self.compiled_step = None
        self.nominal = None
        self._sync_solver()

    def compute(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        if not path or self.backend == "unavailable":
            return ControlCommand()
        if not _is_closed_path(path) and distance_xy(state.pose.x, state.pose.y, path[-1].x, path[-1].y) < 0.18:
            return ControlCommand()

        self._sync_solver()
        self._apply_guidance(state, path)
        self.key, subkey = jax.random.split(self.key)
        state_array = jnp.array([state.pose.x, state.pose.y, state.pose.yaw, state.v, state.steer], dtype=jnp.float32)
        path_x = jnp.array([p.x for p in path], dtype=jnp.float32)
        path_y = jnp.array([p.y for p in path], dtype=jnp.float32)
        path_yaw = jnp.array([p.yaw for p in path], dtype=jnp.float32)

        updated, _costs = self.compiled_step(
            state_array,
            path_x,
            path_y,
            path_yaw,
            self.nominal,
            subkey,
            self.config.target_speed,
            self.config.dt,
            self.config.max_steer,
            self.config.speed_noise_std,
            self.config.steer_noise_std,
            self.config.lambda_mpc,
            self.config.path_weight,
            self.config.heading_weight,
            self.config.goal_weight,
            self.config.control_weight,
            self.config.smooth_weight,
            self.config.progress_weight,
            self.config.speed_weight,
            self.vehicle.wheelbase,
            self.vehicle.max_accel,
            self.vehicle.max_decel,
            self.vehicle.max_steer_rate,
        )
        updated.block_until_ready()
        v0 = float(updated[0, 0])
        steer0 = float(updated[0, 1])
        self.nominal = jnp.concatenate(
            [updated[1:, :], jnp.array([[self.config.target_speed, 0.0]], dtype=jnp.float32)],
            axis=0,
        )
        yaw_rate = self.model.yaw_rate_from_steer(v0, steer0)
        return ControlCommand(v=v0, yaw_rate=yaw_rate, steer=steer0)

    def _sync_solver(self) -> None:
        if self.backend == "unavailable":
            return
        horizon = max(2, int(self.config.horizon_steps))
        batch = max(2, int(self.config.batch_size))
        self.config.horizon_steps = horizon
        self.config.batch_size = batch
        signature = (horizon, batch)
        if self.compiled_signature != signature:
            self.compiled_step = _make_sbmpc_step(horizon, batch)
            self.nominal = jnp.tile(
                jnp.array([[self.config.target_speed, 0.0]], dtype=jnp.float32),
                (horizon, 1),
            )
            self.compiled_signature = signature
        elif self.nominal is None or int(self.nominal.shape[0]) != horizon:
            self.nominal = jnp.tile(
                jnp.array([[self.config.target_speed, 0.0]], dtype=jnp.float32),
                (horizon, 1),
            )

    def _apply_guidance(self, state: VehicleState, path: list[Pose2D]) -> None:
        guide = self._pure_pursuit_guidance(state, path)
        blend = clamp(self.config.guidance_blend, 0.0, 1.0)
        guide_array = jnp.tile(jnp.array([[guide.v, guide.steer]], dtype=jnp.float32), (self.config.horizon_steps, 1))
        self.nominal = (1.0 - blend) * self.nominal + blend * guide_array

    def _pure_pursuit_guidance(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        nearest = min(range(len(path)), key=lambda i: distance_xy(state.pose.x, state.pose.y, path[i].x, path[i].y))
        lookahead = max(self.config.guidance_lookahead, 1e-3)
        target = path[-1]
        for i in range(nearest, len(path)):
            if distance_xy(state.pose.x, state.pose.y, path[i].x, path[i].y) >= lookahead:
                target = path[i]
                break
        heading = math.atan2(target.y - state.pose.y, target.x - state.pose.x)
        alpha = wrap_angle(heading - state.pose.yaw)
        curvature = 2.0 * math.sin(alpha) / lookahead
        yaw_rate = self.config.target_speed * curvature
        steer = self.model.steer_from_yaw_rate(self.config.target_speed, yaw_rate)
        return ControlCommand(v=self.config.target_speed, yaw_rate=yaw_rate, steer=steer)


@dataclass
class StanleyConfig:
    target_speed: float = 0.24
    cross_track_gain: float = 1.2
    heading_gain: float = 1.0
    yaw_rate_limit: float = 0.45


class StanleyController:
    """Stanley-style path tracker for standalone comparison."""

    def __init__(self, config: StanleyConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        self.config = config or StanleyConfig()
        self.vehicle = vehicle or VehicleParams()
        self.model = FourWheelForkliftModel(self.vehicle)

    def compute(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        if not path:
            return ControlCommand()
        front_x = state.pose.x + 0.35 * math.cos(state.pose.yaw)
        front_y = state.pose.y + 0.35 * math.sin(state.pose.yaw)
        nearest_i = min(range(len(path)), key=lambda i: distance_xy(front_x, front_y, path[i].x, path[i].y))
        target = path[nearest_i]
        dx = front_x - target.x
        dy = front_y - target.y
        normal_x = -math.sin(target.yaw)
        normal_y = math.cos(target.yaw)
        cross_track = dx * normal_x + dy * normal_y
        heading_error = wrap_angle(target.yaw - state.pose.yaw)
        desired_yaw_rate = self.config.heading_gain * heading_error - math.atan2(
            self.config.cross_track_gain * cross_track,
            max(abs(self.config.target_speed), 0.05),
        )
        yaw_rate = clamp(desired_yaw_rate, -abs(self.config.yaw_rate_limit), abs(self.config.yaw_rate_limit))
        steer = self.model.steer_from_yaw_rate(self.config.target_speed, yaw_rate)
        return ControlCommand(v=self.config.target_speed, yaw_rate=yaw_rate, steer=steer)


@dataclass
class LQRLiteConfig:
    target_speed: float = 0.28
    lateral_gain: float = 0.9
    heading_gain: float = 1.4
    yaw_rate_limit: float = 0.45


class LQRLiteController:
    """Small hand-tuned LQR-like tracker for standalone comparison."""

    def __init__(self, config: LQRLiteConfig | None = None, vehicle: VehicleParams | None = None) -> None:
        self.config = config or LQRLiteConfig()
        self.vehicle = vehicle or VehicleParams()
        self.model = FourWheelForkliftModel(self.vehicle)

    def compute(self, state: VehicleState, path: list[Pose2D]) -> ControlCommand:
        if not path:
            return ControlCommand()
        nearest_i = min(range(len(path)), key=lambda i: distance_xy(state.pose.x, state.pose.y, path[i].x, path[i].y))
        target = path[nearest_i]
        dx = state.pose.x - target.x
        dy = state.pose.y - target.y
        normal_x = -math.sin(target.yaw)
        normal_y = math.cos(target.yaw)
        lateral_error = dx * normal_x + dy * normal_y
        heading_error = wrap_angle(target.yaw - state.pose.yaw)
        yaw_rate = (
            self.config.heading_gain * heading_error
            + self.config.lateral_gain * lateral_error
        )
        yaw_rate = clamp(yaw_rate, -abs(self.config.yaw_rate_limit), abs(self.config.yaw_rate_limit))
        steer = self.model.steer_from_yaw_rate(self.config.target_speed, yaw_rate)
        return ControlCommand(v=self.config.target_speed, yaw_rate=yaw_rate, steer=steer)
