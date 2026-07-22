def is_terminated(distance_to_goal: float, collision: bool, step_count: int, max_steps: int) -> tuple[bool, bool]:
    terminated = bool(collision) or distance_to_goal < 0.25
    truncated = step_count >= max_steps
    return terminated, truncated
