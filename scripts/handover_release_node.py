#!/usr/bin/env python3

import rospy,actionlib, tf.transformations as ts

from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list, pose_copy

from hr_release.msg import *

def main():

  rospy.init_node('fake_handover_module_test_node')

  vis_cal_cl = actionlib.SimpleActionClient('/vision_system_calibration',VisionSystemCalibrationAction)
  ft_cal_cl = actionlib.SimpleActionClient('/force_torque_sensor_calibration',ForceTorqueSensorCalibrationAction)
  obj_grasp_cl = actionlib.SimpleActionClient('/object_grasp',ObjectGraspAction)
  obj_rec_cl = actionlib.SimpleActionClient('/object_recognition',ObjectRecognitionAction)
  r2h_handv_cl = actionlib.SimpleActionClient('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction)

  ft_cal_cl.wait_for_server()
  obj_grasp_cl.wait_for_server()
  obj_rec_cl.wait_for_server()
  r2h_handv_cl.wait_for_server()

  # set position for vision_calibration
  vis_cal_goal = VisionSystemCalibrationGoal()
  vis_cal_goal.home.position.x = -0.650
  vis_cal_goal.home.position.y = -0.109
  vis_cal_goal.home.position.z = 0.223
  vis_cal_goal.home.orientation = list_to_quat([0,0,-0.7,0.7])
  vis_cal_goal.max_accel = 0.8
  vis_cal_goal.max_angaccel = 0.8
  vis_cal_goal.delta = 0.2

  # set position for sensor_calibration
  ft_cal_goal = ForceTorqueSensorCalibrationGoal()
  ft_cal_goal.home.position.x = -0.486
  ft_cal_goal.home.position.y = -0.109
  ft_cal_goal.home.position.z = 0.223
  ft_cal_goal.home.orientation = list_to_quat([0,0,-0.7,0.7])
  ft_cal_goal.max_accel = 0.8
  ft_cal_goal.max_angaccel = 1.0

  # grasp the object
  obj_grasp_goal = ObjectGraspGoal()
  obj_grasp_goal.tool_to_obj.position.x = -0.254
  obj_grasp_goal.tool_to_obj.position.y = -0.137
  obj_grasp_goal.tool_to_obj.position.z = -0.020
  obj_grasp_goal.tool_to_obj.orientation.x = -0.690
  obj_grasp_goal.tool_to_obj.orientation.y = 0.163
  obj_grasp_goal.tool_to_obj.orientation.z = -0.144
  obj_grasp_goal.tool_to_obj.orientation.w = 0.690
  obj_grasp_goal.hand_preshape.data = [0.1,0.4,0.0]
  obj_grasp_goal.hand_target.data = [0.5,1.0,1.1]
  obj_grasp_goal.delta.z = 0.15
  obj_grasp_goal.max_accel = 0.4
  obj_grasp_goal.max_angaccel = 0.4
  obj_grasp_goal.max_vel = 0.4
  obj_grasp_goal.max_angvel = 0.4
  obj_grasp_goal.alpha = 0.2

  # start object_recognition while positioning for reaching
  obj_rec_goal = ObjectRecognitionGoal()
  obj_rec_goal.home = pose_copy(ft_cal_goal.home)
  obj_rec_goal.max_accel = 0.6
  obj_rec_goal.max_angaccel = 0.6

  # continue with reaching
  r2h_handv_goal = RobotHumanHandoverReachingGoal()
  r2h_handv_goal.target.position.x = -0.577
  r2h_handv_goal.target.position.y = -0.267
  r2h_handv_goal.target.position.z = 0.328
  r2h_handv_goal.target.orientation.x = 0.850
  r2h_handv_goal.target.orientation.y = -0.281
  r2h_handv_goal.target.orientation.z = -0.335
  r2h_handv_goal.target.orientation.w = 0.290
  r2h_handv_goal.back = pose_copy(ft_cal_goal.home)
  r2h_handv_goal.max_accel = 0.4
  r2h_handv_goal.max_angaccel = 0.4
  r2h_handv_goal.stop_time = 0.2
  r2h_handv_goal.sleep = 1

  # high level control loop
  while True:

    # start or check vision_calibration
    vis_cal_cl.send_goal_and_wait(vis_cal_goal)

    # start or check ft sensor calibration
    # ft_cal_cl.send_goal_and_wait(ft_cal_goal)
    
    # grasp
    obj_grasp_cl.send_goal_and_wait(obj_grasp_goal)
    
    # do object recognition
    # obj_rec_cl.send_goal_and_wait(obj_rec_goal)

    # continue with the handover
    r2h_handv_cl.send_goal_and_wait(r2h_handv_goal)

    break

  rospy.loginfo('bye!')

  # rospy.spin()

if __name__ == '__main__':
  main()