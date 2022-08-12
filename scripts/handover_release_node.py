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
  obj_grasp_goal.tool_to_obj.position.x = -0.044
  obj_grasp_goal.tool_to_obj.position.y = 0.137
  obj_grasp_goal.tool_to_obj.position.z = -0.310
  obj_grasp_goal.tool_to_obj.orientation.x = 0.325
  obj_grasp_goal.tool_to_obj.orientation.y = -0.246
  obj_grasp_goal.tool_to_obj.orientation.z = 0.633
  obj_grasp_goal.tool_to_obj.orientation.w = 0.658
  obj_grasp_goal.hand_preshape.data = [0.1,0.4,0.0]
  obj_grasp_goal.hand_target.data = [0.4,0.75,0.5]
  obj_grasp_goal.delta.z = 0.1
  obj_grasp_goal.max_accel = 0.8
  obj_grasp_goal.max_angaccel = 0.8
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
  r2h_handv_goal.back = pose_copy(ft_cal_goal.home)
  r2h_handv_goal.target_off.position.x = -0.251
  r2h_handv_goal.target_off.position.y = -0.591
  r2h_handv_goal.target_off.position.z = 0.359
  r2h_handv_goal.target_off.orientation.x = 0.755
  r2h_handv_goal.target_off.orientation.y = -0.224
  r2h_handv_goal.target_off.orientation.z = -0.170
  r2h_handv_goal.target_off.orientation.w = 0.591
  r2h_handv_goal.max_accel = 0.2
  r2h_handv_goal.max_angaccel = 0.2
  r2h_handv_goal.stop_time = 0.5
  r2h_handv_goal.sleep = 1
  r2h_handv_goal.hand_open_pos.data = [0.1,0.4,0.0]
  r2h_handv_goal.release_type = r2h_handv_goal.FIXED
  r2h_handv_goal.release_duration = 1.0 # used only if release type is FIXED

  # high level control loop
  while True:

    # start or check vision_calibration
    #vis_cal_cl.send_goal_and_wait(vis_cal_goal)

    # start or check ft sensor calibration
    ft_cal_cl.send_goal_and_wait(ft_cal_goal)
    
    # grasp
    obj_grasp_cl.send_goal_and_wait(obj_grasp_goal)
    
    # do object recognition
    #obj_rec_cl.send_goal_and_wait(obj_rec_goal)

    # continue with the handover
    r2h_handv_cl.send_goal_and_wait(r2h_handv_goal)

    break

  rospy.loginfo('bye!')

  # rospy.spin()

if __name__ == '__main__':
  main()