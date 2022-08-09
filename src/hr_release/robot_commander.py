from abc import ABC

import rospy

from artificial_hands_py.artificial_hands_py_base import singleton
from artificial_hands_py.robot_commander import RobotCommanderBase

from hr_release.wrist_dynamics_base import WristDynamics
from hr_release.vision_system_manager_base import VisionSystem

class RobotCommander(RobotCommanderBase,ABC):

  def __init__(self) -> None:
    super().__init__()

    self.vision = VisionSystem() 

    self.wrist_dyn = WristDynamics()

def main():

  rospy.init_node('robot_commnader_node')

  robot = RobotCommander()

  rospy.loginfo('Robot commander_ready!')

  rospy.spin()

if __name__ == '__main__':
  main()