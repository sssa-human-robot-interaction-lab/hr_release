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

    self.rec_peak_acc = 0 # this value stores receiver peak acceleration
    self.open_dur = 0.5 # this value controls the release duration
    self.distance = 0 # this value stores distance to target

    self.r2h_handv_as.start()

  def r2h_handover_cb(self, goal : RobotHumanHandoverReachingGoal):
    self.r2h_handv_result.success = True
    self.r2h_handv_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)

    # start to store proprioceptive information to further thresholding, but first wait a bit to get filter adapt to calib offset
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.set_save_interaction()
    
    # set arm control
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_poly_567_traj_generator()
    self.arm.switch_to_cartesian_controller(self.controller)
    
    # go to the target position
    self.target = goal.target_off
    self.arm.set_pose_target(self.target,False)
    self.update_target_state()

    # reaching fixed distance to target, then start to trigger
    rate = rospy.Rate(30)
    while self.distance > 0.2:
      self.update_target_state()
      rate.sleep()  
    self.wrist_dyn.set_trigger_dynamics()

    # set release type and prepare hand to open in ad-hoc separate thread
    if goal.release_type == goal.ADAPTIVE:
      open_dur = self.open_dur
    elif goal.release_type == goal.FIXED:
      open_dur = goal.release_duration

    open_thread = Thread(target=self.release,args=[goal.hand_open_pos.data,open_dur])
    
    # update online target according to human hand pose, stop arm at contact time
    self.wrist_dyn.detection.dynamic_contact = False
    while not self.wrist_dyn.detection.dynamic_contact:  
      self.update_target_state()
      rate.sleep() 
    open_thread.start()
    self.arm.stop(wait = False)
      
    # wait until hand is open
    open_thread.join()

    # wait a bit before set wrist_dynamics_module to idle
    rospy.sleep(rospy.Duration(goal.sleep))
    self.wrist_dyn.stop_loop()

    # wait a bit and retire to back position
    rospy.sleep(rospy.Duration(goal.sleep))
    self.arm.set_poly_567_traj_generator()
    self.arm.switch_to_cartesian_controller(self.controller)
    self.arm.set_pose_target(goal.back)

    # stop controllers
    rospy.sleep(self.sleep_dur)
    self.arm.pause_all_controllers()
    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)
  
  def update_target_state(self):
    
    self.lock.acquire()

    c_position = self.arm.get_current_frame().pose.position
    self.distance = pow(pow(c_position.x - self.target.position.x,2)+pow(c_position.y - self.target.position.y,2)+pow(c_position.z - self.target.position.z,2),0.5)
  
    # x = self.vision.rec_pnt.get_acceleration_norm()
    # if x > self.rec_peak_acc:
    #   self.rec_peak_acc = x
    #   self.open_dur = (3.0*x*x - 11.5*x + 260)/1000
    
    self.r2h_handv_feedback.distance_to_target = self.distance
    self.r2h_handv_as.publish_feedback(self.r2h_handv_feedback)

    self.lock.release()

  def release(self, pos, dur):
    rospy.logwarn('Releasing object!')
    self.hand.set_joint_target_positions(pos, dur)

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo('R2H handover reaching module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()