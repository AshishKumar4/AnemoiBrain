from gym.envs.registration import register


register(id = 'airsim-v0', entry_point='gym_airsim.envs:AirSimEnv',)
register(id = 'airsim-extrahard-v0', entry_point='gym_airsim.envs:AirSimExtraHardEnv',)

