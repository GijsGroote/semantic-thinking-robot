from gijsRobot.envs.gijsRobotEnv import GijsRobotEnv


class GijsRobotVelEnv(GijsRobotEnv):

    def applyAction(self, action):
        self.robot.apply_vel_action(action)

    def setSpaces(self):
        (self.observation_space, self.action_space) = self.robot.getVelSpaces()
