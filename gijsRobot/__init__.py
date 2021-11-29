from gym.envs.registration import register
register(
    id='gijsRobot-vel-v0',
    entry_point='gijsRobot.envs:GijsRobotVelEnv'
)
register(
    id='gijsRobot-acc-v0',
    entry_point='gijsRobot.envs:GijsRobotAccEnv'
)
