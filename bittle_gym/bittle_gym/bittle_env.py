import math
import threading
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

import rclpy
from bittle_gym.gazebo_connector import GazeboConnector
from bittle_gym.bittle_controller import BittleController

NUM_LEGS = 4
NUM_JOINTS_FOR_LEG = 2

SHOULDER_LOWER_VALUE, SHOULDER_UPPER_VALUE = -1.5708, 1.22173
KNEE_LOWER_VALUE, KNEE_UPPER_VALUE = -1.22173, 1.48353


class BittleEnv(gym.Env):
    """
    Custom Environment that follows gym interface.
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()

        # Gazebo
        self.gazebo = GazeboConnector()

        gazebo_spin_thread = threading.Thread(target=rclpy.spin, args=(self.gazebo,))
        gazebo_spin_thread.start()

        # Bittle
        joint_names = [
            "left_back_shoulder_joint",
            "left_back_knee_joint",
            "left_front_shoulder_joint",
            "left_front_knee_joint",
            "right_back_shoulder_joint",
            "right_back_knee_joint",
            "right_front_shoulder_joint",
            "right_front_knee_joint"
        ]
        self.bittle = BittleController(joint_names)

        bittle_spin_thread = threading.Thread(target=rclpy.spin, args=(self.bittle,))
        bittle_spin_thread.start()


        # Action space
        self.action_space = gym.spaces.Box(
            low=np.array([
                [SHOULDER_LOWER_VALUE, KNEE_LOWER_VALUE],
                [SHOULDER_LOWER_VALUE, KNEE_LOWER_VALUE],
                [SHOULDER_LOWER_VALUE, KNEE_LOWER_VALUE],
                [SHOULDER_LOWER_VALUE, KNEE_LOWER_VALUE],
            ]), 
            high=np.array([
                [SHOULDER_UPPER_VALUE, KNEE_UPPER_VALUE],
                [SHOULDER_UPPER_VALUE, KNEE_UPPER_VALUE],
                [SHOULDER_UPPER_VALUE, KNEE_UPPER_VALUE],
                [SHOULDER_UPPER_VALUE, KNEE_UPPER_VALUE],
            ]),
            shape=(NUM_LEGS, 2), 
            dtype=np.float64
        )
        print(self.action_space.sample())

        # Observation space
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(2, 8), 
            dtype=np.float64
        )
        print(self.observation_space.sample())


        self.score = 0
        self.terminated, self.truncated = False, False

        self.target_joint_states = np.array([
            [1., 1., 1., 1., 1., 1., 1., 1.],
            [0., 0., 0., 0., 0., 0., 0., 0.],
        ])


    def step(self, action):
        self.bittle.pub_joint_cmd_pos(action)
        self.gazebo.step()
        observation = self.take_observation()
        
        # if self.ep_timestep > 1000:
        #     self.terminated = True
        # else :
        #     self.ep_timestep += 1

        # reward = 0
        # if self.terminated or self.truncated:
        reward = self.take_reward(observation)
        self.score += reward

        self.terminated = self.check_termination(observation)
        
        info = {}
        return observation, reward, self.terminated, self.truncated, info

    def reset(self, seed=None, options=None):
        self.score = 0
        self.gazebo.reset(seed)
        observation = self.take_observation()
        self.terminated, self.truncated = False, False
        info = {}
        return observation, info

    def render(self):
        return

    def close(self):
        return


    def take_observation(self):
        """
        Take observation from the environment
        """
        observation = self.bittle.joint_states_observation
        return observation

    def take_reward(self, observation):
        """
        """
        diff = self.target_joint_states - observation
        max = np.max(diff[0])
        mae = np.abs(diff[0]).mean()
        mse = np.square(diff[0]).mean()

        T = 1.
        reward = 1. * math.exp(-mae/T) - 0.2
        # max = np.max(np.square(diff[0]))
        # reward = 10 * math.exp(-max) - 1
        return reward

    def check_termination(self, observation, treshold=0.02):
        diff = self.target_joint_states - observation
        mae = np.abs(diff[0]).mean()
        if mae < treshold:
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)

    env = BittleEnv()
    env.reset()
    check_env(env)

    # Model (Proximal Policy Optimization)
    model = PPO("MlpPolicy", env, verbose=1)

    # Training Loop
    episodes = 30
    for ep in range(episodes):
        TIMESTEPS = 10000
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False)
        print(f"score: {env.score}")

    env.close()


if __name__ == '__main__':
    main()
