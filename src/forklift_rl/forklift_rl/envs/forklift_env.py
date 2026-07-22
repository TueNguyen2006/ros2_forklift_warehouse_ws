import math
import numpy as np

from forklift_rl.envs.observation_builder import ObservationBuilder
from forklift_rl.envs.reward import shaped_reward
from forklift_rl.envs.simulator_stepper import SimulatorStepper
from forklift_rl.envs.spaces import spaces
from forklift_rl.envs.termination import is_terminated


class ForkliftEnv:
    metadata = {"render_modes": []}

    def __init__(self, *, max_steps: int = 500, lidar_bins: int = 24, seed: int | None = None):
        self.max_steps = max_steps
        self.builder = ObservationBuilder(lidar_bins=lidar_bins)
        self.stepper = SimulatorStepper()
        self.rng = np.random.default_rng(seed)
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.step_count = 0
        self.action_space = spaces.Box(
            low=np.asarray([-0.35, -0.55], dtype=np.float32),
            high=np.asarray([0.75, 0.55], dtype=np.float32),
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.builder.size,),
            dtype=np.float32,
        )

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.step_count = 0
        self.previous_action = np.zeros(2, dtype=np.float32)
        obs = self._observation(distance_to_goal=3.0, heading_error=0.0)
        info = {"seed": seed, "randomization": (options or {}).get("randomization", {})}
        return obs, info

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        self.stepper.step(action)
        self.step_count += 1
        distance_to_goal = max(0.0, 3.0 - 0.02 * self.step_count)
        heading_error = 0.0
        collision = False
        obs = self._observation(distance_to_goal=distance_to_goal, heading_error=heading_error)
        reward = shaped_reward(distance_to_goal, heading_error, collision)
        terminated, truncated = is_terminated(distance_to_goal, collision, self.step_count, self.max_steps)
        self.previous_action = action
        info = {"distance_to_goal": distance_to_goal, "collision": collision}
        return obs, reward, terminated, truncated, info

    def _observation(self, *, distance_to_goal: float, heading_error: float):
        return self.builder.build(
            relative_goal_x=distance_to_goal,
            relative_goal_y=0.0,
            heading_error=heading_error,
            cross_track_error=0.0,
            current_velocity=float(self.previous_action[0]),
            current_steering_angle=float(self.previous_action[1]),
            previous_action=self.previous_action,
            lidar_ranges=np.full(self.builder.lidar_bins, 12.0, dtype=np.float32),
        )
