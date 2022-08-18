from hr_release.rosbag_manager_base import ROSBagManager
from hr_release.hr_release_gui.hr_release_gui_base import *

class HandoverReleaseExperimentCondition(QWidget, ROSBagManager):

  def __init__(self, block_name : str, id : int):
    super().__init__()

    self.id = id
    self.name = block_name + "_{}".format(str(id))

    id_label = QLabel('#{}'.format(str(id)))
    id_label.setAlignment(Qt.AlignCenter | Qt.AlignLeft)
    
    self.store_push_button = QPushButton('Store',self)
    self.reset_push_button = QPushButton('Reset',self)

    self.grasp_push_button = ResultButton('Grasp',self,self.set_reach_enabled)
    self.reach_push_button = ResultButton('Reach',self,self.set_store_enabled)

    self.grasp_push_button.clicked.connect(self.set_disabled)
    self.reach_push_button.clicked.connect(self.set_disabled)
    
    self.reach_push_button.clicked.connect(self.on_start_condition)
    self.store_push_button.clicked.connect(self.on_store_condition)
    self.reset_push_button.clicked.connect(self.on_reset_condition)

    self.set_disabled()

    layout = QHBoxLayout()
    layout.addWidget(id_label)
    layout.addWidget(self.grasp_push_button)
    layout.addWidget(self.reach_push_button)
    layout.addWidget(self.store_push_button)
    layout.addWidget(self.reset_push_button)

    self.setLayout(layout)
  
  def set_disabled(self):
    self.store_push_button.setDisabled(True)
    self.reset_push_button.setDisabled(True)
    self.grasp_push_button.setDisabled(True)
    self.reach_push_button.setDisabled(True)

  def set_reach_enabled(self):
    if self.grasp_push_button.result:
      self.reach_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)

  def set_store_enabled(self):
    if self.reach_push_button.result:
      self.store_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)
  
  def on_start_condition(self, id):
    self.bag_name = self.bag_open(self.name)

  def on_store_condition(self):
    if confirm_dialog(self, 'Store?'):
      self.store_push_button.setDisabled(True)

  def on_reset_condition(self):
    if confirm_dialog(self, 'Reset?'):
      self.bag_remove(self.bag_name)
      self.set_disabled()
      self.grasp_push_button.reset()
      self.reach_push_button.reset()
      self.grasp_push_button.setEnabled(True)

  @pyqtSlot(int)
  def on_changed_result_button(self, value):
    self.bag_close()

class HandoverReleaseExperimentBlock(QWidget):

  def __init__(self, parent, block_name : str, block_repetitions : int):
    super().__init__(parent)

    self.block_name = block_name

    self.conditions = []
    for id in range(block_repetitions):
      self.conditions.append(HandoverReleaseExperimentCondition(block_name,id))
    
    self.conditions[0].grasp_push_button.setEnabled(True)

    layout = QVBoxLayout()
    for condition in self.conditions:
      layout.addWidget(condition)
    
    self.setLayout(layout)
  
  def on_next_condition(self, id):
    self.conditions[id+1].grasp_push_button.setEnabled(True)
  
  