import gymnasium as gym
import numpy as np
from gazebo_connector import GazeboConnector
from bittle_controller import BittleController

NUM_LEGS = 4
NUM_JOINTS_FOR_LEG = 2

SHOULDER_MIN_VALUE, SHOULDER_MAX_VALUE = -1.0, 1.0
ELBOW_MIN_VALUE, ELBOW_MAX_VALUE = -1.0, 1.0


class BittleEnv(gym.Env):
    """
    Custom Environment that follows gym interface.
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()

        # Gazebo
        self.gazebo = GazeboConnector()

        # Bittle
        self.bittle = BittleController()

        
        # Define action and observation space
        self.action_space = gym.spaces.Box(
            low=np.array([SHOULDER_MIN_VALUE, ELBOW_MIN_VALUE]), 
            high=np.array([SHOULDER_MAX_VALUE, ELBOW_MAX_VALUE]), 
            shape=(NUM_LEGS, 2), 
            dtype=np.float32
        )
        print(self.action_space.sample())
        self.observation_space = gym.spaces.Box(
            shape=(2, 6), 
        )


    def step(self, action):
        self.gazebo.step()
        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        self.gazebo.reset(seed)
        return observation, info

    # def render(self):
    #     ...

    # def close(self):
    #     ...

    def take_observations(self):

        
        data_imu