import logging
from gym.envs.registration import register
from gymGreenWave.GreenWaveEnv import GreenWaveEnv

logger = logging.getLogger(__name__)

register(
    id='gymGreenWave-v0',
    entry_point='gymGreenWave:GreenWaveEnv',
    kwargs={'oneway' : True, 'uneven': False, 'GUI': True},
)

register(
    id='gymGreenWave-v1',
    entry_point='gymGreenWave:GreenWaveEnv',
    kwargs={'oneway' : True, 'uneven': False, 'GUI': False},
)
