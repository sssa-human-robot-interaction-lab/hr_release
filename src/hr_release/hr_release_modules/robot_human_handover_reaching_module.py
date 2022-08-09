from threading import Thread

import rospy, actionlib

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from hr_release.msg import *
from hr_release.robot_commander import RobotCommander

class RobotHumanHandoverReachingModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  r2h_handv_feedback = RobotHumanHandoverReachingFeedback()
  r2h_handv_result = RobotHumanHandoverReachingResult()

  def __init__(self) -> None:
    super().__init__()
  
    self.r2h_handv_as = actionlib.SimpleActionServer('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)

    trg_sub = rospy.Subscriber("handover_target_point",CartesianTrajectoryPoint,callback=self.handover_target_cb)
    
    # this value controls the release duration
    self.open_vel : float

    # start by sending the actual frame to online trajectory generator
    self.target = self.arm.get_current_frame().pose

    self.r2h_handv_as.start()

  def handover_target_cb(self, msg : CartesianTrajectoryPoint):
    self.target = msg.pose 

  def r2h_handover_cb(self, goal : RobotHumanHandoverReachingGoal):
    self.r2h_handv_result.success = True
    self.r2h_handv_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)

    # start to store proprioceptive information to further thresholding, but first wait a bit to get filter adapt to calib offset
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.set_save_interaction()
    
    # move to start position
    self.arm.set_track_ratio(1)
    self.arm.set_track_t_go(2)
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_max_accel(0.2) # use lower acceleration to reach the home position
    self.arm.set_max_angaccel(0.2)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.home)

    # change to online taget generator
    self.arm.set_mj_traj_generator()
    rospy.sleep(self.sleep_dur)
    
    # reaching fixed distance to target threshold, then start to trigger
    rate = rospy.Rate(30)
    while self.distance > 1e-2 and self.distance < 0.4:
      self.arm.set_pose_target(goal.target,False)
      self.compute_target_distance()
      rate.sleep()  
    self.wrist_dyn.set_trigger_dynamics()

    # prepare hand to open in a separate thread
    self.hand.switch_to_controller('mia_hand_joint_group_vel_controller')
    open_thread = Thread(target=self.release,args=[False])
    open_thread.start()
    
    # update online target according to human hand pose, stop arm when near to target
    while self.distance > 0.2: 
      self.arm.set_pose_target(goal.target,False)
      self.compute_target_distance()
      rate.sleep() 
    self.arm.stop(wait = False)
      
    # wait until hand is open
    open_thread.join()

    # wait a bit before set wrist_dynamics_module to idle
    rospy.sleep(rospy.Duration(goal.sleep))
    self.wrist_dyn.stop_loop()

    # wait a bit and retire to back position
    rospy.sleep(rospy.Duration(goal.sleep))
    self.arm.set_max_accel(0.2) # use lower accelration to reach the back position
    self.arm.set_max_angaccel(0.2)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.back)

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)

  def compute_target_distance(self):
    c_position = self.arm.get_current_frame().pose
    self.distance = pow(pow(c_position.x - self.target.position.x,2)+pow(c_position.y - self.target.position.y,2),pow(c_position.y - self.target.position.y,2),.5)

  def release(self):
    rate = rospy.Rate(100)
    self.wrist_dyn.detection.dynamic_contact = False
    while not self.wrist_dyn.detection.dynamic_contact and not rospy.is_shutdown():
      rate.sleep()
    self.hand.open(self.open_vel)

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo('R2H handover reaching module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()