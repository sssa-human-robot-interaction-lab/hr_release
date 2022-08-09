#!/usr/bin/env python3

import rospy

from hr_release.hr_release_modules.vision_system_calibration_module import VisionSystemCalibrationModule
from hr_release.hr_release_modules.force_torque_sensor_calibration_module import ForceTorqueSensorCalibrationModule
from hr_release.hr_release_modules.object_grasp_module import ObjectGraspModule
from hr_release.hr_release_modules.object_recognition_module import ObjectRecognitionModule
from hr_release.hr_release_modules.robot_human_handover_reaching_module import RobotHumanHandoverReachingModule

def main():

  rospy.init_node('fake_low_level_engine_node')

  vis_cal_mod = VisionSystemCalibrationModule()
  ft_cal_mod = ForceTorqueSensorCalibrationModule()
  obj_grasp_mod = ObjectGraspModule()
  obj_rev_mod = ObjectRecognitionModule()
  r2h_handv_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo("Low-level control engine ready!")

  rospy.spin()

if __name__ == '__main__':
  main()