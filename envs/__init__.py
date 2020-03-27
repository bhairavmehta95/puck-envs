from gym.envs.registration import register
from envs.pucks import PucksEnv

register(
    id='Pucks-v0',
    entry_point='envs.pucks:PucksEnv',
    max_episode_steps=5000,
    kwargs={'frame_skip': 10}
)

