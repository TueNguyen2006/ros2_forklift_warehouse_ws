# forklift_rl

Planned home for project-owned reinforcement learning environments, observations, rewards, agents, curriculum, and evaluation adapters.

The upstream RL environment remains unchanged under `src/third_party/ROS2-Forklift-Simulation/src/forklift_gym_env`. Do not modify upstream source directly; add project-specific wrappers or adapters here once their interface is defined.

Expected future layout:

```text
envs/
observations/
rewards/
agents/
curriculum/
evaluation/
```
