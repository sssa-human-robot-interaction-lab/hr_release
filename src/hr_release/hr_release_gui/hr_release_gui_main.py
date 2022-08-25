from hr_release.hr_release_gui.hr_release_gui_base import *

class HandoverReleaseExperimentMainControl(QWidget):

  def __init__(self, parent):
    super().__init__(parent)

    self.new_subject_button = QPushButton('New subject',self)
    self.terminate_subject_button = QPushButton('Terminate subject',self)
    
    self.calib_vision_button = QResultButton('Calibrate vision',self,self.set_enabled)
    self.calib_sensor_button = QResultButton('Calibrate sensor',self,self.set_enabled)
    self.object_grasp_button = QResultButton('Grasp object',self,self.set_enabled)
    self.object_recog_button = QResultButton('Object recognition',self,self.set_enabled)
    self.reach_fast_button = QResultButton('Reach fast',self,self.set_enabled)
    self.reach_slow_button = QResultButton('Reach slow',self,self.set_enabled)
    
    self.reach_fast_button.set_button_rgb([0,255,0,50])
    self.reach_slow_button.set_button_rgb([0,255,0,50])

    #self.set_disabled()

    self.calib_vision_button.setEnabled(True)
    
    self.calib_vision_button.clicked.connect(self.set_disabled)
    self.calib_sensor_button.clicked.connect(self.set_disabled)
    self.object_grasp_button.clicked.connect(self.set_disabled)
    self.object_recog_button.clicked.connect(self.set_disabled)
    self.reach_fast_button.clicked.connect(self.set_disabled)
    self.reach_slow_button.clicked.connect(self.set_disabled)
    self.object_recog_button.clicked.connect(self.set_disabled)

    self.subject_name_line = QLineEdit()
    self.subject_name_line.setPlaceholderText("Mario Rossi")

    self.subject_age_spinbox = QSpinBox()
    self.subject_age_spinbox.setValue(30)
    self.subject_age_spinbox.setSingleStep(1)

    self.subject_gender_spinbox = QComboBox()
    self.subject_gender_spinbox.addItem('M')
    self.subject_gender_spinbox.addItem('F')

    subject_name_label = QLabel('Subject:')
    subject_name_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    subject_age_label = QLabel('Age:')
    subject_age_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    subject_layout = QHBoxLayout()
    subject_layout.addWidget(subject_name_label)
    subject_layout.addWidget(self.subject_name_line)
    subject_layout.addWidget(subject_age_label)
    subject_layout.addWidget(self.subject_age_spinbox)
    subject_layout.addWidget(self.subject_gender_spinbox)
    
    subject_buttons_layout = QHBoxLayout()
    subject_buttons_layout.addWidget(self.new_subject_button)
    subject_buttons_layout.addWidget(self.terminate_subject_button)

    system_layout = QVBoxLayout()
    system_layout.addWidget(self.calib_vision_button)
    system_layout.addWidget(self.calib_sensor_button)
    system_layout.addWidget(self.object_grasp_button)
    system_layout.addWidget(self.object_recog_button)
    system_layout.addWidget(self.reach_slow_button)
    system_layout.addWidget(self.reach_fast_button)

    system_group_box = QGroupBox('System control')
    system_group_box.setLayout(system_layout)

    main_layout = QVBoxLayout()
    main_layout.addLayout(subject_layout)
    main_layout.addLayout(subject_buttons_layout)
    main_layout.addWidget(system_group_box)

    self.setLayout(main_layout)
  
  def set_disabled(self):
    self.new_subject_button.setDisabled(True)
    self.terminate_subject_button.setDisabled(True)
    self.calib_vision_button.setDisabled(True)
    self.calib_sensor_button.setDisabled(True)
    self.object_grasp_button.setDisabled(True)
    self.object_recog_button.setDisabled(True)
    self.reach_fast_button.setDisabled(True)
    self.reach_slow_button.setDisabled(True)
      
  def set_enabled(self):
    self.calib_vision_button.setEnabled(True)
    self.calib_sensor_button.setEnabled(True)
    if self.calib_sensor_button.success and self.calib_vision_button.success:
      self.object_grasp_button.setEnabled(True)
      if self.object_grasp_button.success:
        self.object_recog_button.setEnabled(True)
        if self.object_recog_button.success:
          self.new_subject_button.setEnabled(True)
          self.terminate_subject_button.setEnabled(True) 
          self.reach_fast_button.setEnabled(True)
          self.reach_slow_button.setEnabled(True)