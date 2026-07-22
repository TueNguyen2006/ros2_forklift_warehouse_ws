import numpy as np


class ObservationBuilder:
    def __init__(self, lidar_bins: int = 24) -> None:
        self.lidar_bins = lidar_bins

    @property
    def size(self) -> int:
        return 8 + self.lidar_bins

    def build(
        self,
        *,
        relative_goal_x: float,
        relative_goal_y: float,
        heading_error: float,
        cross_track_error: float,
        current_velocity: float,
        current_steering_angle: float,
        previous_action,
        lidar_ranges,
    ) -> np.ndarray:
        lidar = np.asarray(lidar_ranges, dtype=np.float32)
        if lidar.size == 0:
            lidar = np.full(self.lidar_bins, 12.0, dtype=np.float32)
        if lidar.size != self.lidar_bins:
            indices = np.linspace(0, lidar.size - 1, self.lidar_bins).astype(int)
            lidar = lidar[indices]
        action = np.asarray(previous_action, dtype=np.float32)
        return np.concatenate(
            [
                np.asarray(
                    [
                        relative_goal_x,
                        relative_goal_y,
                        heading_error,
                        cross_track_error,
                        current_velocity,
                        current_steering_angle,
                    ],
                    dtype=np.float32,
                ),
                action,
                lidar.astype(np.float32),
            ]
        )
