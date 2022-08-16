#!/usr/bin/env python3

import rospy

from hr_release.hr_execution_modules.vision_system_calibration_module import VisionSystemCalibrationModule
from hr_release.hr_execution_modules.force_torque_sensor_calibration_module import ForceTorqueSensorCalibrationModule
from hr_release.hr_execution_modules.object_grasp_module import ObjectGraspModule
from hr_release.hr_execution_modules.object_recognition_module import ObjectRecognitionModule
from hr_release.hr_execution_modules.robot_human_handover_reaching_offline_module import RobotHumanHandoverReachingModule

def main():

  rospy.init_node('hr_execution_modules_node')

  cart_controller = 'cartesian_eik_velocity_controller'

  vis_cal_mod = VisionSystemCalibrationModule(cart_controller)
  ft_cal_mod = ForceTorqueSensorCalibrationModule(cart_controller)
  obj_grasp_mod = ObjectGraspModule(cart_controller)
  obj_rec_mod = ObjectRecognitionModule(cart_controller)
  r2h_handv_mod = RobotHumanHandoverReachingModule(cart_controller)

  rospy.loginfo("Execution modules ready!")

  rospy.spin()

if __name__ == '__main__':
  main()