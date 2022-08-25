import sys, random

from shutil import rmtree
from os import remove

import rospy,actionlib

from geometry_msgs.msg import TransformStamped
from artificial_hands_msgs.msg import *
from mia_hand_msgs.msg import Mia
from ur_msgs.msg import Analog

from hr_release.msg import *
from hr_release.hr_release_gui.hr_release_gui_main import *
from hr_release.hr_release_gui.hr_release_gui_block import *

class HandoverReleaseExperimentGUI(QROSUtils):

  bag_dir = '/home/penzo/Desktop'

  bag_dict = {'wrist_dynamics_data' : WristDynamicsStamped, 
  'wrist_contact_detection' : DetectionStamped,
  'mia_hand/mia' : Mia, 
  'vicon/Receiver/receiver' : TransformStamped,
  'vicon_state/Receiver/receiver' : CartesianTrajectoryPointStamped,
  'io_states_stamped' : IOStatesStamped,
  'robot_to_human_handover_reaching_offline/feedback' : RobotHumanHandoverReachingActionFeedback,
  }

  r2h_timing = {'slow' : 1.0, 'fast' : 2.0}
  r2h_release = {'slow' : 0.4, 'fast' : 0.0, 'adaptive' : -1.0}
  
  block_dict = {'A1' : {'timing' : 'slow', 'release' : 'adaptive'},
  'F11' : {'timing' : 'slow', 'release' : 'slow'},
  'F12' : {'timing' : 'slow', 'release' : 'fast'},
  'A2' : {'timing' : 'fast', 'release' : 'adaptive'},
  'F21' : {'timing' : 'fast', 'release' : 'slow'},
  'F22' : {'timing' : 'fast', 'release' : 'fast'}}

  def __init__(self, goals : dict) -> None:
    super().__init__()

    self.bag.bag_create(self.bag_dict)

    self.goals = goals

    self.vis_cal_cl = actionlib.SimpleActionClient('/vision_system_calibration',VisionSystemCalibrationAction)
    self.ft_cal_cl = actionlib.SimpleActionClient('/force_torque_sensor_calibration',ForceTorqueSensorCalibrationAction)
    self.obj_grasp_cl = actionlib.SimpleActionClient('/object_grasp',ObjectGraspAction)
    self.obj_rec_cl = actionlib.SimpleActionClient('/object_recognition',ObjectRecognitionAction)
    self.r2h_handv_cl = actionlib.SimpleActionClient('/robot_to_human_handover_reaching_offline',RobotHumanHandoverReachingAction)

    self.ft_cal_cl.wait_for_server()
    self.obj_grasp_cl.wait_for_server()
    self.obj_rec_cl.wait_for_server()
    self.r2h_handv_cl.wait_for_server()

    self.blocks = []
    self.blocks_id = []

    self.main = HandoverReleaseExperimentMainControl(self)

    self.tab_widget = QTabWidget(self)
    self.tab_widget.addTab(self.main,'Main')

    main_layout = QHBoxLayout()
    main_layout.addWidget(self.tab_widget)

    self.main.new_subject_button.clicked.connect(self.on_new_subject)
    self.main.terminate_subject_button.clicked.connect(self.on_terminate_subject)

    self.main.calib_sensor_button.clicked.connect(self.on_calib_sensor)
    self.main.calib_vision_button.clicked.connect(self.on_calib_vision)
    self.main.object_grasp_button.clicked.connect(partial(self.on_grasp_object, self.main.object_grasp_button))
    self.main.object_recog_button.clicked.connect(self.on_rec_object)
    self.main.reach_fast_button.clicked.connect(partial(self.on_reach_to_handover, self.main.reach_fast_button, 'A2'))
    self.main.reach_slow_button.clicked.connect(partial(self.on_reach_to_handover, self.main.reach_slow_button, 'A1'))
    
    self.setLayout(main_layout)

  def on_new_subject(self):

    if self.main.subject_name_line.text() == '':
      self.main.subject_name_line.setText('Unkown')

    if q_confirm_dialog(self,"Create new subject {}?".format(self.main.subject_name_line.text())):

      self.block_scores_dict =  {'A1' : 0, 'F11' : 0, 'F12' : 0, 'A2' : 0, 'F21' : 0, 'F22' : 0}
      self.block_confirm_dict =  {'A1' : False, 'F11' : False, 'F12' : False, 'A2' : False, 'F21' : False, 'F22' : False}

      block_names = list(self.block_dict.keys())
      block_order = ''
      random.shuffle(block_names)
      for block_name in block_names:
        self.blocks.append(HandoverReleaseExperimentBlock(self,block_name,6))
        block_order += block_name + ','
      
      block : HandoverReleaseExperimentBlock
      for block in self.blocks:
        id = self.tab_widget.addTab(block,block.block_name)
        self.blocks_id.append(id)
        block.score_confirmed.connect(self.on_score_confirmed)
        block.next_block_button.clicked.connect(partial(self.on_next_block,id))
      
      for block in self.blocks:
        condition : HandoverReleaseExperimentCondition
        for condition in block.conditions:
          condition.grasp_push_button.clicked.connect(partial(self.on_grasp_object,condition.grasp_push_button))
          condition.reach_push_button.clicked.connect(partial(self.on_reach_to_handover,condition.reach_push_button,block.block_name))
      
      for block in self.blocks[1:]:
        block.setDisabled(True)

      readme = "{name}\n{age}\n{gend}\n{order}".format(name = self.main.subject_name_line.text(), 
      age = str(self.main.subject_age_spinbox.value()), gend = self.main.subject_gender_spinbox.currentText(),
      order = block_order)
      
      self.subject_dir = self.bag.bag_mkdir(self.bag_dir+'/{}'.format(self.main.subject_name_line.text()),readme)

      analog_out_0 : Analog = self.io_states.analog_out[0]
      while analog_out_0.domain != Analog.VOLTAGE or analog_out_0.state < 4.0:
        q_info_dialog(self,'Check analog output!')
        analog_out_0 = self.io_states.analog_out[0]

      analog_in_0 : Analog = self.io_states.analog_in[0]
      analog_in_1 : Analog = self.io_states.analog_in[1]
      while analog_in_0.domain != Analog.VOLTAGE or analog_in_1.domain != Analog.VOLTAGE:
        q_info_dialog(self,'Check analog input!')
        analog_in_0 = self.io_states.analog_in[0]
        analog_in_1 = self.io_states.analog_in[1]
  
  def closeEvent(self, event):
    if self.on_terminate_subject():
        event.accept() 
    else:
        event.ignore()

  def on_next_block(self,id):
    self.blocks[id-1].setDisabled(True)
    if id >= len(self.blocks):
      self.tab_widget.setCurrentIndex(1)
    else:
      self.blocks[id].setEnabled(True)
      self.tab_widget.setCurrentIndex(id+1)
  
  @pyqtSlot(dict)
  def on_score_confirmed(self, score_dict):
    with open(self.subject_dir+'/SCORES_active.txt','a') as f:
      f.write("{name},{score},".format(name=score_dict['block_name'],score=score_dict['score']))
    self.block_scores_dict[score_dict['block_name']] = score_dict['score']
    self.block_confirm_dict[score_dict['block_name']] = True
  
  def terminate_subject(self):
    
    self.main.calib_vision_button.reset()
    self.main.calib_sensor_button.reset()
    self.main.object_grasp_button.reset()
    self.main.object_recog_button.reset()

    self.blocks_id.reverse()
    for id in self.blocks_id:
      self.tab_widget.removeTab(id)
    self.blocks_id.clear()
    self.blocks.clear()

    del self.block_scores_dict, self.block_confirm_dict

  def on_terminate_subject(self):

    try:
      self.block_scores_dict.values()
    except:
      return True

    if not all(self.block_scores_dict.values()):
      if q_confirm_dialog(self,"Not all blocks are confirmed, discard subject {}?".format(self.main.subject_name_line.text())):
        rmtree(self.subject_dir)
        self.terminate_subject()
        return True

    if q_confirm_dialog(self,"Terminate subject {}?".format(self.main.subject_name_line.text())):

      with open(self.subject_dir+'/SCORES.txt','w') as f:
        for block_name in self.block_scores_dict.keys():
          if self.block_confirm_dict[block_name]:
            f.write("{name},{score},".format(name=block_name,score=self.block_scores_dict[block_name]))
      
      remove(self.subject_dir+'/SCORES_active.txt')
   
      self.terminate_subject()

      return True
    
    return False

  def on_calib_vision(self):
    vis_cal_goal = self.goals['vis_cal']
    self.vis_cal_cl.send_goal(vis_cal_goal,done_cb = self.main.calib_vision_button.set_result_button,
    feedback_cb = self.main.calib_vision_button.set_progress_button)
    
  def on_calib_sensor(self):
    ft_cal_goal = self.goals['ft_cal']
    self.ft_cal_cl.send_goal(ft_cal_goal,done_cb = self.main.calib_sensor_button.set_result_button,
    feedback_cb = self.main.calib_sensor_button.set_progress_button)  
  
  def on_grasp_object(self, button : QResultButton):
    obj_grasp_goal = self.goals['obj_grasp']
    self.obj_grasp_cl.send_goal(obj_grasp_goal,done_cb = button.set_result_button,
    feedback_cb = button.set_progress_button)
  
  def on_rec_object(self):
    obj_rec_goal = self.goals['obj_rec']
    self.obj_rec_cl.send_goal(obj_rec_goal,done_cb = self.main.object_recog_button.set_result_button,
    feedback_cb = self.main.object_recog_button.set_progress_button)
  
  def on_reach_to_handover(self, button : QResultButton, block_name : str):
    r2h_handv_goal = self.goals['r2h_handv']
    r2h_handv_goal : RobotHumanHandoverReachingGoal

    block = self.block_dict[block_name]

    r2h_handv_goal.max_accel = self.r2h_timing[block['timing']]
    r2h_handv_goal.max_angaccel = self.r2h_timing[block['timing']]
    
    r2h_handv_goal.release_duration = self.r2h_release[block['release']] #used only if release_type is FIXED

    if block['release'] == 'adaptive':
      r2h_handv_goal.release_type = r2h_handv_goal.ADAPTIVE
    else:
      r2h_handv_goal.release_type = r2h_handv_goal.FIXED

    self.r2h_handv_cl.send_goal(r2h_handv_goal,done_cb = button.set_result_button,
    feedback_cb = button.set_progress_button)

def main():
  
  """ Set constants """

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
  obj_grasp_goal.hand_preshape.data = [0.2,0.6,0.0]
  obj_grasp_goal.hand_target.data = [0.45,0.9,0.4]
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

  # continue with offline reaching
  r2h_handv_goal = RobotHumanHandoverReachingGoal()
  r2h_handv_goal.back = pose_copy(ft_cal_goal.home)
  r2h_handv_goal.target_off.position.x = -0.251
  r2h_handv_goal.target_off.position.y = -0.591
  r2h_handv_goal.target_off.position.z = 0.359
  r2h_handv_goal.target_off.orientation.x = 0.755
  r2h_handv_goal.target_off.orientation.y = -0.224
  r2h_handv_goal.target_off.orientation.z = -0.170
  r2h_handv_goal.target_off.orientation.w = 0.591
  r2h_handv_goal.stop_time = 0.5
  r2h_handv_goal.sleep = 1
  r2h_handv_goal.hand_open_pos.data = [0.2,0.6,0.0]

  """ Initialize node and GUI """

  rospy.init_node('handover_release_experiment_gui_node')

  app = QApplication(sys.argv)

  hr_rel_exp_gui = HandoverReleaseExperimentGUI(goals =
   {'vis_cal' : vis_cal_goal,
    'ft_cal' : ft_cal_goal, 
    'obj_grasp' : obj_grasp_goal, 
    'obj_rec' : obj_rec_goal, 
    'r2h_handv' : r2h_handv_goal})
  
  hr_rel_exp_gui.show()

  rospy.loginfo('Handover release experiment GUI ready!')

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()

