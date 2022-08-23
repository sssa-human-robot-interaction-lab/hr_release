from multiprocessing.connection import wait
import numpy as np

import rospy, actionlib, tf.transformations as ts

from artificial_hands_py.artificial_hands_py_base import *

from hr_release.msg import *
from hr_release.robot_commander import RobotCommander

class ObjectGraspModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  grasp_feedback = ObjectGraspFeedback()
  grasp_result = ObjectGraspResult()

  def __init__(self, cartesian_controller : str = 'cartesian_eik_position_controller') -> None: 
    super().__init__()
    
    self.controller = cartesian_controller
    
    self.grasp_as = actionlib.SimpleActionServer('/object_grasp',ObjectGraspAction,execute_cb=self.grasp_cb,auto_start=False)

    self.grasp_as.start()

  def grasp_cb(self, goal : ObjectGraspGoal):
    self.grasp_result.success = True
    self.grasp_feedback.percentage = 0

    # get hand in open position
    self.hand.set_joint_target_positions([0.2,0.6,0.0],0.6)

    # set arm control
    self.arm.switch_to_cartesian_controller(self.controller)
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_poly_345_traj_generator()
    
    # retrieve target in the reference frame and move arm
    T_OT = pose_to_matrix(goal.tool_to_obj)
    T_BO = pose_to_matrix(self.vision.obj_pnt.get_pose())
    T = np.dot(T_BO,T_OT)
    h_pose = matrix_to_pose(T)
    self.arm.set_pose_target(h_pose)
    self.grasp_feedback.percentage += 30
    self.grasp_as.publish_feedback(self.grasp_feedback)

    #get hand in pre-shape
    self.hand.set_joint_target_positions(goal.hand_preshape.data,0.6)

    # fix target pose for grasp
    t_pose = pose_copy(h_pose)
    t_pose.position.x -= goal.delta.x
    t_pose.position.y -= goal.delta.y
    t_pose.position.z -= goal.delta.z

    # go to grasp pose with trapz
    self.arm.set_alpha(goal.alpha)
    self.arm.set_max_vel(goal.max_vel)
    self.arm.set_max_angvel(goal.max_angvel)
    self.arm.set_mod_trapz_traj_generator()
    self.arm.set_pose_target(t_pose,wait = False)
    self.arm.wait_for_trajectory_monitor(99)
    self.grasp_feedback.percentage += 30
    self.grasp_as.publish_feedback(self.grasp_feedback)

    # get hand in closed shape and stop control
    self.hand.set_joint_target_positions(target = goal.hand_target.data, goal_time = 1.0)
    # rospy.sleep(self.sleep_dur)
    self.hand.stop()

    # go to back position
    self.arm.set_pose_target(h_pose)
    self.grasp_feedback.percentage += 30
    self.grasp_as.publish_feedback(self.grasp_feedback)

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.grasp_as.set_succeeded(self.grasp_result)

def main():

  rospy.init_node('object_grasp_module_node')

  obj_rec_mod = ObjectGraspModule()

  rospy.loginfo('Object grasp module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()