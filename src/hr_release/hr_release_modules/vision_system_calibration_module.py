from cmath import pi

import rospy, actionlib
import tf.transformations as ts
from geometry_msgs.msg import Pose

from artificial_hands_py.artificial_hands_py_base import pose_to_list, pose_copy

from hr_release.msg import *
from hr_release.robot_commander import RobotCommander

class VisionSystemCalibrationModule(RobotCommander):
  sleep_dur = rospy.Duration(1)
  calib_feedback = VisionSystemCalibrationFeedback()
  calib_result = VisionSystemCalibrationResult()

  def __init__(self) -> None: 
    super().__init__()
  
    self.calib_as = actionlib.SimpleActionServer('/vision_system_calibration',VisionSystemCalibrationAction,execute_cb=self.calibration_cb,auto_start=False)

    self.calib_as.start()

  def calibration_cb(self, goal : VisionSystemCalibrationGoal):
    self.calib_result.success = True
    self.calib_feedback.calib_ok = False
    self.calib_feedback.percentage = 40

    # get hand in open position
    self.hand.set_joint_positions([0.4,0.4,0.3])

    # go to home position
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.home)
    rospy.sleep(self.sleep_dur)

    # reset previous calibration
    self.vision.reset_calibration_matrix()
    rospy.sleep(self.sleep_dur)

    # get current pose and init list for four point calibration
    o_pose = self.arm.get_current_frame(frame = 'probe').pose
    pnts = [self.vision.giv_pnt.get_position()]
    
    # move +x
    c_pose = self.arm.get_current_frame().pose
    c_pose.position.x += goal.delta
    self.arm.set_pose_target(c_pose)
    rospy.sleep(self.sleep_dur)
    pnts.append(self.vision.giv_pnt.get_position())

    # move +y
    c_pose.position.y += goal.delta
    self.arm.set_pose_target(c_pose)
    rospy.sleep(self.sleep_dur)
    pnts.append(self.vision.giv_pnt.get_position())

    # return to home position
    self.arm.set_pose_target(goal.home)

    # perform four point calibration
    self.vision.four_point_calibration(pnts,pose_to_list(o_pose))

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.calib_as.set_succeeded(self.calib_result)

def main():

  rospy.init_node('vision_system_calibration_module_node')

  vis_cal_mod = VisionSystemCalibrationModule()

  rospy.loginfo('Vision system calibration module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()