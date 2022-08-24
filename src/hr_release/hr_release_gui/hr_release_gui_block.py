from hr_release.rosbag_manager_base import QROSBagManager
from hr_release.hr_release_gui.hr_release_gui_base import *

class HandoverReleaseExperimentCondition(QROSBagManager):

  stored = pyqtSignal(int)
  unstored = pyqtSignal(int)

  def __init__(self, block_name : str, id : int):
    super().__init__()

    self.id = id
    self.name = block_name + "_{}".format(str(id))

    id_label = QLabel('#{}'.format(str(id)))
    id_label.setAlignment(Qt.AlignCenter | Qt.AlignLeft)
    
    self.store_push_button = QPushButton('Store',self)
    self.reset_push_button = QPushButton('Reset',self)

    self.grasp_push_button = QResultButton('Grasp',self,self.set_reach_enabled)
    self.reach_push_button = QResultButton('Reach',self,self.set_store_enabled)

    self.grasp_push_button.clicked.connect(self.set_disabled)
    self.reach_push_button.clicked.connect(self.set_disabled)
    
    self.reach_push_button.clicked.connect(self.on_start_condition)
    self.store_push_button.clicked.connect(self.on_store_condition)
    self.reset_push_button.clicked.connect(self.on_reset_condition)

    self.reach_push_button.changed.connect(self.on_changed_result_button)

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
    if self.grasp_push_button.success:
      self.reach_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)

  def set_store_enabled(self):
    if self.reach_push_button.success:
      self.store_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)
  
  def on_start_condition(self, id):
    self.bag_name = self.bag.bag_open(self.name)

  def on_store_condition(self):
    if q_confirm_dialog(self, 'Store?'):
      self.store_push_button.setDisabled(True)
      self.stored.emit(self.id)

  def on_reset_condition(self):
    if q_confirm_dialog(self, 'Reset?'):
      if self.reach_push_button.result:
        self.bag.bag_remove(self.bag_name)
      self.set_disabled()
      self.grasp_push_button.reset()
      self.reach_push_button.reset()
      self.grasp_push_button.setEnabled(True)
      self.unstored.emit(self.id)

  @pyqtSlot()
  def on_changed_result_button(self):
    self.bag.bag_close()

class HandoverReleaseExperimentBlock(QWidget):

  score_confirmed = pyqtSignal(dict)

  def __init__(self, parent, block_name : str, block_repetitions : int):
    super().__init__(parent)

    self.block_name = block_name
    self.block_score = 0

    self.conditions = []
    self.conditions_stored = [False]*block_repetitions

    for id in range(block_repetitions):
      self.conditions.append(HandoverReleaseExperimentCondition(block_name,id))
    
    self.conditions[0].grasp_push_button.setEnabled(True)

    layout = QVBoxLayout()
    condition : HandoverReleaseExperimentCondition
    for condition in self.conditions:
      condition.stored.connect(self.on_stored_condition)
      condition.unstored.connect(self.on_unstored_condition)
      layout.addWidget(condition)

    self.next_block_button = QPushButton('Next')
    self.score_confirm_button = QPushButton('Confirm')

    self.score_confirm_button.clicked.connect(self.on_confirm_score)

    self.score_combo_box = QComboBox()
    self.score_combo_box.addItem('-3 (delay)')
    for score in range(-2,3):
      self.score_combo_box.addItem(str(score))
    self.score_combo_box.addItem('3 (early)')

    score_label = QLabel('Block score:')
    score_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    
    self.score_layout = QHBoxLayout()
    self.score_layout.addWidget(score_label)
    self.score_layout.addWidget(self.score_combo_box)
    self.score_layout.addWidget(self.score_confirm_button)
    self.score_layout.addWidget(self.next_block_button)
    
    self.set_rate_disabled()

    layout.addLayout(self.score_layout)
    
    self.setLayout(layout)

  def set_rate_enabled(self):
    self.score_combo_box.setEnabled(True)
    self.score_confirm_button.setEnabled(True)

  def set_rate_disabled(self):
    self.score_combo_box.setDisabled(True)
    self.score_confirm_button.setDisabled(True)
    self.next_block_button.setDisabled(True)
  
  def on_confirm_score(self):
    if q_confirm_dialog(self,'Confirm score {}'.format(self.score_combo_box.currentText())):
      
      condition : HandoverReleaseExperimentCondition
      for condition in self.conditions:
        condition.set_disabled()

      self.score_combo_box.setDisabled(True)
      self.score_confirm_button.setDisabled(True)
      self.next_block_button.setEnabled(True)

      score_dict = {'block_name' : self.block_name, 'score' : self.score_combo_box.currentText()}
      self.score_confirmed.emit(score_dict)

  @pyqtSlot(int)
  def on_stored_condition(self, id):
    self.conditions_stored[id] = True
    if all(self.conditions_stored):
      self.set_rate_enabled()
    else:
      self.conditions[id+1].grasp_push_button.setEnabled(True)
    

  @pyqtSlot(int)
  def on_unstored_condition(self, id):
    self.conditions_stored[id] = False
