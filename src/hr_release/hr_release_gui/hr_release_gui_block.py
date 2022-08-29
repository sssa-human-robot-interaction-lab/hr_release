from hr_release.hr_release_gui.hr_release_gui_base import *

class HandoverReleaseExperimentCondition(QROSUtils):

  confirmed = pyqtSignal(int,int)
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

    trial_layout = QHBoxLayout()
    trial_layout.addWidget(id_label)
    trial_layout.addWidget(self.grasp_push_button)
    trial_layout.addWidget(self.reach_push_button)
    trial_layout.addWidget(self.store_push_button)
    trial_layout.addWidget(self.reset_push_button)

    self.score_confirm_button = QPushButton('Confirm')
    self.score_confirm_button.clicked.connect(self.on_score_confirmed)

    self.score_combo_box = QComboBox()
    self.score_combo_box.addItem('-3 (delay)')
    for score in range(-2,3):
      self.score_combo_box.addItem(str(score))
    self.score_combo_box.addItem('3 (early)')

    score_label = QLabel('Score:')
    score_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    
    score_layout = QHBoxLayout()
    score_layout.addWidget(score_label)
    score_layout.addWidget(self.score_combo_box)
    score_layout.addWidget(self.score_confirm_button)

    self.set_disabled()

    layout = QVBoxLayout()
    layout.addLayout(trial_layout)
    layout.addLayout(score_layout)

    self.setLayout(layout)
  
  def set_rate_enabled(self):
    self.score_combo_box.setEnabled(True)
    self.score_confirm_button.setEnabled(True)

  def set_rate_disabled(self):
    self.score_combo_box.setDisabled(True)
    self.score_confirm_button.setDisabled(True)
  
  def set_disabled(self):
    self.store_push_button.setDisabled(True)
    self.reset_push_button.setDisabled(True)
    self.grasp_push_button.setDisabled(True)
    self.reach_push_button.setDisabled(True)
    
    self.set_rate_disabled()

  def set_reach_enabled(self):
    if self.grasp_push_button.success:
      self.reach_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)

  def set_store_enabled(self):
    if self.reach_push_button.success:
      self.store_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)
  
  def on_start_condition(self, id):
    self.io_states.reset()
    self.bag_name = self.bag.bag_open(self.name)

  def on_store_condition(self):
    if not self.io_states.analog_in_active:
      q_info_dialog(self, 'Analog in never activated, please repeat!')
    else:
      self.set_rate_enabled()
    self.store_push_button.setDisabled(True)

  def on_reset_condition(self):
    if q_confirm_dialog(self, 'Reset?'):
      if self.reach_push_button.result:
        self.bag.bag_remove(self.bag_name)
      self.set_disabled()
      self.grasp_push_button.reset()
      self.reach_push_button.reset()
      self.grasp_push_button.setEnabled(True)
      self.unstored.emit(self.id)
  
  def on_score_confirmed(self):
    self.set_rate_disabled()
    score : str = self.score_combo_box.currentText()
    score = score.replace(' (delay)','')
    score = score.replace(' (early)','')
    self.confirmed.emit(self.id,int(score))

  @pyqtSlot()
  def on_changed_result_button(self):
    self.bag.bag_close()

class HandoverReleaseExperimentBlock(QWidget):

  completed = pyqtSignal(dict)

  def __init__(self, parent, block_name : str, block_repetitions : int):
    super().__init__(parent)

    self.block_name = block_name
    self.block_score = 0

    self.conditions = []
    self.conditions_scores = [0]*block_repetitions
    self.conditions_stored = [False]*block_repetitions

    for id in range(block_repetitions):
      self.conditions.append(HandoverReleaseExperimentCondition(block_name,id))
    
    self.conditions[0].grasp_push_button.setEnabled(True)

    layout = QVBoxLayout()
    condition : HandoverReleaseExperimentCondition
    for condition in self.conditions:
      condition.confirmed.connect(self.on_score_confirmed)
      condition.unstored.connect(self.on_unstored_condition)
      layout.addWidget(condition)
    
    self.setLayout(layout)
  
  @pyqtSlot(int,int)
  def on_score_confirmed(self, id, score):
    self.conditions_stored[id] = True
    self.conditions_scores[id] = score
    if all(self.conditions_stored):
      self.completed.emit({'block_name' : self.block_name, 'scores' : self.conditions_scores})
    else:
      self.conditions[id+1].grasp_push_button.setEnabled(True)

  @pyqtSlot(int)
  def on_unstored_condition(self, id):
    self.conditions_stored[id] = False
