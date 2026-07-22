import math


def shaped_reward(distance_to_goal: float, heading_error: float, collision: bool = False) -> float:
    reward = -float(distance_to_goal) - 0.1 * abs(float(heading_error))
    if collision:
        reward -= 25.0
    if distance_to_goal < 0.25:
        reward += 20.0
    return reward
