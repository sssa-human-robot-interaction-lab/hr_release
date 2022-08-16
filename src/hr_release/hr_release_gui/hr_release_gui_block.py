from hr_release.hr_release_gui.hr_release_gui_base import *

class HandoverReleaseExperimentCondition(QWidget):

  def __init__(self, id : int):
    super().__init__()

    self.id = id

    id_label = QLabel('#{}'.format(str(id)))
    id_label.setAlignment(Qt.AlignCenter | Qt.AlignLeft)
    
    self.store_push_button = QPushButton('Store',self)
    self.reset_push_button = QPushButton('Reset',self)

    self.grasp_push_button = ResultButton('Grasp',self,self.set_enabled)
    self.reach_push_button = ResultButton('Reach',self,self.set_enabled)

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

  def set_enabled(self):
    if self.grasp_push_button.result:
      self.reach_push_button.setEnabled(True)
      self.reset_push_button.setEnabled(True)
      if self.reach_push_button.result:
        self.store_push_button.setEnabled(True)

class HandoverReleaseExperimentBlock(QWidget):

  def __init__(self, parent, block_name : str, block_repetitions : int):
    super().__init__(parent)

    self.block_name = block_name

    self.conditions = []
    for c in range(1,block_repetitions+1):
      self.conditions.append(HandoverReleaseExperimentCondition(c))

    condition : HandoverReleaseExperimentCondition
    for condition in self.conditions:
      condition.store_push_button.clicked.connect(partial(self.on_store_condition,condition.id))
    
    self.conditions[0].grasp_push_button.setEnabled(True)

    layout = QVBoxLayout()
    for condition in self.conditions:
      layout.addWidget(condition)
    
    self.setLayout(layout)
  
  def on_store_condition(self, id):
    if confirm_dialog('Store?'):
      self.conditions[id+1].grasp_push_button.setEnabled(True)