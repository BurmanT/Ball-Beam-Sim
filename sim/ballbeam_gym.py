import gymnasium as gym
import pybullet as p
import numpy as np
from typing import Optional
from pybullet_utils import bullet_client

class BallBeamGym(gym.Env):
    '''
    Gym environment for BallBeamBot
    '''

    metadata = {"render.modes": ["human"]}

    def __init__(self, agent, actions, random_generator, setpoint, beam_length,render = False):
        '''
        Setup Gym environment, start pybullet and call reset

        the provided constructor argument "render" determines wheter pybullet is run headlessly
        '''
        self.agent = agent
        self.render = render

        self.action_space = gym.spaces.Discrete([0.1, 0, -0.1])

        self.current_angle = None

        # position, change in position, angle of beam 
        self.observation_space = gym.spaces.box.Box(
            low=np.array([0, -1, -0.5]),
            high=np.array([1, 1, 0.5]),
        )

        self.total_sum_reward_tracker = []
        self.total_timestep_tracker = []

        self.episode_reward_tracker = []

        self.random_generator = random_generator
        self.goal = setpoint
        self.beam_length = beam_length

        self.bot = None

        self.count = 0
        self.reset()

    def step(self, action):
        '''
        Take action and return observation

        :param action: action to take
        '''
        complete = False

        if self.current_angle is None:
            self.current_angle = self.bot.get_current_angle()

        self.bot.apply_action(action)
        obs = self.bot.measure_distance()

        p.stepSimulation()

        self.count += 1
        if self.count >= 2000:
            complete = True
            self.count = 0

        reward = (1-abs(self.goal-obs)/self.beam_length)^2

        self.episode_reward_tracker.append(reward)

        if complete: 
            self.collect_statistics()
    
        return np.array(obs, dtype=np.float32), reward, complete, False, {}


    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        '''
        Reset robots posistion and goal posistion randomly
        '''
        
        # Spawn robot randomly
        self.bot = self.agent(self.render)

        dist = self.bot.measure_distance()

        return np.array(dist, dtype=np.float32), {}

    def close(self):
        '''
        Close 
        '''

        self.collect_statistics()

    def collect_statistics(self) -> None:
        '''
        collect statistics function is used to record total sum and total timesteps per episode
        '''
        self.total_sum_reward_tracker.append(sum(self.episode_reward_tracker))
        self.total_timestep_tracker.append(len(self.episode_reward_tracker))

        self.episode_reward_tracker = []