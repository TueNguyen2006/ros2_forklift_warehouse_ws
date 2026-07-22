class SimulatorStepper:
    """Abstraction for deterministic Gazebo stepping.

    Gazebo Classic deployments differ in synchronous stepping support. This
    class intentionally centralizes stepping so the environment does not spread
    timing logic or `sleep()` calls through training code.
    """

    def __init__(self, physics_iterations_per_step: int = 10) -> None:
        self.physics_iterations_per_step = physics_iterations_per_step

    def pause(self) -> None:
        pass

    def publish_action(self, action) -> None:
        self.last_action = action

    def step_iterations(self) -> None:
        # Placeholder for Gazebo stepping service/plugin integration. Until a
        # synchronous stepping API is wired, smoke tests use this no-op.
        pass

    def read_state(self) -> dict:
        return {}

    def step(self, action) -> dict:
        self.pause()
        self.publish_action(action)
        self.step_iterations()
        return self.read_state()
