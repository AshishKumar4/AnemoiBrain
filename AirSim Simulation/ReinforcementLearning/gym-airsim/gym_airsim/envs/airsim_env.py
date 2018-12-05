import gym
from gym import error, spaces, utils 
from gym.utils import seeding 
from gym.spaces import *
from gym.spaces.box import Box

class AirSimEnv(gym.Env):
    metadata = {'render.modes':['human']}
    def __init__(self):
        # First we create an Observation Space. As an Observation, We gather
        # a Disparity Map of the enviornment along with a raw float image, and
        # two arrays (each for x and y axis) to denote the target direction in the vieport.

        # For now, for simplicity, I am just taking two arrays of float numbers, 
        # storing the direction of the target.
        self.observation_space = spaces.Box(low = 0, high = 1, shape = (1, 3), dtype=float) # 2 numbers to represent a coordinate in space of the plane perpendicular to the movement.
        
        # Then we create our Action Space. Our Action space is a continuous 1D vector space, 
        # 4 vectors for throttle, pitch, roll, yaw; range from -1 to 1. Throttle would be converted to 0 to 2 by the engine later.
        self.action_space = spaces.Box(low = -1, high = -1, shape = (4, ))
    def reward(self):
        ... 
    def step(self, action):
        ... 
    def reset(self):
        ... 
    def render(self, mode='human', close = False):
        ...