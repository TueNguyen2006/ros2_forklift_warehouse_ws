import unittest

from forklift_rl.envs.forklift_env import ForkliftEnv


class ForkliftEnvTest(unittest.TestCase):
    def test_reset_and_step_shapes_match_spaces(self):
        env = ForkliftEnv(lidar_bins=8, seed=7)
        obs, info = env.reset(seed=7)

        self.assertEqual(obs.shape, env.observation_space.shape)
        self.assertEqual(env.action_space.shape, (2,))
        self.assertEqual(info["seed"], 7)

        next_obs, reward, terminated, truncated, step_info = env.step([0.1, 0.05])
        self.assertEqual(next_obs.shape, env.observation_space.shape)
        self.assertIsInstance(reward, float)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertIn("distance_to_goal", step_info)
