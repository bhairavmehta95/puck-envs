#/usr/bin/env python
# manual

"""
This script allows you to manually control the simulator or Duckiebot
using the keyboard arrows.
"""

import sys
from pynput import keyboard
from pynput.keyboard import Key
import numpy as np
import gym
import envs

env = gym.make('Pucks-v0')

reward = 0.

env.reset()
env.render()

ACTIONS = [
        np.array([1.0, 0.0]),
        np.array([-1.0, 0.0]),
        np.array([0.0, 1.0]),
        np.array([0.0, -1.0]),
        np.array([0.0, 0.0])
]

ACTION_KEYS = [Key.right, Key.left, Key.up, Key.down, Key.page_down]

i = 0 
action = [0.1, 0.0, 0, 0]
while True:
    s_, r, d, _ = env.step(action)
    env.render()
    i+=1
    if i < 1000:
        print(i, s_)

    # Simulates (effectively) elastic collisions 
    if not np.array_equal(s_[-2:], [0, 0]):
        action = [0.0, 0.0, 0.1, 0]

    if d: 
        env.reset()
        env.render()


env.close()
