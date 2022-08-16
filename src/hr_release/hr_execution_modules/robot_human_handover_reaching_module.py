from threading import Thread, Lock

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

  lock = Lock()

  def __init__(self, cartesian_controller : str = 'cartesian_eik_position_controller') -> None: 
    super().__init__()
    
    self.controller = cartesian_controller
  
    self.r2h_handv_as = actionlib.SimpleActionServer('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)

    self.rec_peak_acc : float # this value stores receiver peak acceleration
    self.open_dur : float # this value controls the release duration
    self.distance = 0 # this value stores distance to target

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
    # self.wrist_dyn.set_save_interaction()
    
    # start from current pose, set arm control
    self.arm.set_track_ratio(1)
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_max_accel(0.2) # use lower acceleration to go back
    self.arm.set_max_angaccel(0.2)
    #self.arm.switch_to_cartesian_controller(self.controller)
    
    # change to online taget generator
    self.arm.set_mj_traj_generator()
    rospy.sleep(self.sleep_dur)
    
    # reaching fixed distance to target, then start to trigger (for mj keep decresaing the t_go)
    c = 0
    self.rec_peak_acc = 0 # reset receiver peak acceleration
    rate = rospy.Rate(30)
    dt = rate.sleep_dur.to_sec()
    while self.distance < 0.4:
      self.arm.set_track_t_go(goal.goal_time - c*dt)
      self.target = self.vision.rec_pnt.get_pose()
      self.arm.set_pose_target(self.target,False)
      self.update_target_state()
      rate.sleep()  
      c += 1
    # self.wrist_dyn.set_trigger_dynamics()

    # prepare hand to open in a separate thread
    open_thread = Thread(target=self.release,args=[goal.hand_open_pos,])
    open_thread.start()
    
    # update online target according to human hand pose, stop arm when near to target or at contact time
    while self.distance > 0.2 and not self.wrist_dyn.detection.dynamic_contact:  
      self.arm.set_track_t_go(goal.goal_time - c*dt)
      self.target = self.vision.rec_pnt.get_pose()
      self.arm.set_pose_target(self.target,False)
      self.update_target_state()
      rate.sleep() 
      c += 1
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
    # self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.back)

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)
  
  def update_target_state(self):
    
    self.lock.acquire()

    c_position = self.arm.get_current_frame().pose.position
    self.distance = pow(pow(c_position.x - self.target.position.x,2)+pow(c_position.y - self.target.position.y,2)+pow(c_position.z - self.target.position.y,2),0.5)
    
    x = self.vision.rec_pnt.get_acceleration_norm()
    if x > self.rec_peak_acc:
      self.rec_peak_acc = x
      self.open_dur = (3.0*x*x - 11.5*x + 260)/1000

    self.lock.release()

  def release(self, pos):
    
    rate = rospy.Rate(100)
    
    self.wrist_dyn.detection.dynamic_contact = False
    while not self.wrist_dyn.detection.dynamic_contact and not rospy.is_shutdown():
      rate.sleep()
    
    self.hand.set_joint_target_positions(pos, self.open_dur)

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo('R2H handover reaching module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()