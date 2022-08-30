from threading import Thread, Lock

import rospy, actionlib

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
  
    self.r2h_handv_as = actionlib.SimpleActionServer('/robot_to_human_handover_reaching_offline',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)
    self.r2h_handv_as.start()

  def r2h_handover_cb(self, goal : RobotHumanHandoverReachingGoal):
    self.r2h_handv_result.success = True
    self.r2h_handv_feedback.percentage = 0

    self.rec_peak_acc = 0.0 # this value stores receiver peak acceleration
    self.open_dur = 0.0 # this value controls the release duration
    self.distance = 0.0 # this value stores distance to target

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)

    # start to estimate proprioceptive informations and then apply zero
    self.wrist_dyn.set_estimate_wrench()
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.do_zero()

    # start to store proprioceptive information for further thresholding, but first wait a bit to get filter adapt to zero
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.set_save_interaction()
    
    # set arm control
    self.arm.switch_to_cartesian_controller(self.controller)
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_poly_345_traj_generator()
    self.arm.switch_to_cartesian_controller(self.controller)
    
    # go to the target position
    self.target = goal.target_off
    self.arm.set_pose_target(self.target,False)
    self.update_receiver_state()
    self.r2h_handv_feedback.percentage += 25
    self.r2h_handv_as.publish_feedback(self.r2h_handv_feedback)

    # reaching fixed distance from receiver hand, then start to trigger
    rate = rospy.Rate(100)
    while self.distance > 0.5:
      self.update_receiver_state()
      rate.sleep()  
    self.wrist_dyn.set_trigger_dynamics()

    # set release type and prepare hand to open in ad-hoc separate thread
    release_thread = Thread(target=self.release,args=[goal.release_type,goal.release_duration,goal.hand_open_pos.data])
    self.hand.switch_to_controller(self.hand.mia_j_vel_ctrl)

    # update online target according to human hand pose, stop arm at contact time
    self.wrist_dyn.detection.dynamic_contact = False
    while not self.wrist_dyn.detection.dynamic_contact:  
      self.update_receiver_state()
      rate.sleep() 
    release_thread.start()
    self.arm.stop(wait = False)
    self.r2h_handv_feedback.percentage += 25
    self.r2h_handv_as.publish_feedback(self.r2h_handv_feedback)
      
    # wait until hand is open
    release_thread.join()

    # wait a bit before set wrist_dynamics_module to idle and retire to back position
    rospy.sleep(rospy.Duration(goal.sleep))
    self.wrist_dyn.stop_loop()
    self.r2h_handv_feedback.percentage += 25
    self.r2h_handv_as.publish_feedback(self.r2h_handv_feedback)
    self.arm.set_poly_567_traj_generator()
    self.arm.switch_to_cartesian_controller(self.controller)
    self.arm.set_pose_target(goal.back,wait_tf=True)

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)
  
  def update_receiver_state(self):

    self.lock.acquire()

    receiver_pos = self.vision.rec_pnt.get_position()
    c_position = self.arm.get_current_frame().pose.position
    self.distance = pow(pow(c_position.x - receiver_pos[0],2)+pow(c_position.y - receiver_pos[1],2)+pow(c_position.z - receiver_pos[2],2),0.5)
    
    x = self.vision.rec_pnt.get_acceleration_norm()
    if x > self.rec_peak_acc and x < 8.0 and self.distance < 1.0:
      self.rec_peak_acc = x
      m = 4.261*x*x -67.96*x + 420.3 # receiver acceleration to release duration 
      self.open_dur = -1.249e-05*m*m + 0.008259*m - 0.7545 #release duration to hand open goal time
      if self.open_dur < 0.2:
        self.open_dur = 0.2

    self.r2h_handv_feedback.receiver_accel_norm = x
    self.r2h_handv_feedback.receiver_peak_accel = self.rec_peak_acc
    self.r2h_handv_feedback.distance_to_target = self.distance
    self.r2h_handv_as.publish_feedback(self.r2h_handv_feedback)

    self.lock.release()    

  def release(self, release_type : int, release_duration : float, target_pos : list):
    if release_type == RobotHumanHandoverReachingGoal.ADAPTIVE:   
      open_dur = self.open_dur 
    elif release_type == RobotHumanHandoverReachingGoal.FIXED:
      open_dur = release_duration
    self.hand.set_joint_target_positions(target_pos, open_dur, vel_interface = True)
    rospy.logwarn('Releasing object: goal time {:.2f} s, peak acc {:.2f} m/s2'.format(open_dur,self.rec_peak_acc))

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo('R2H handover reaching module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()
