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

    self.set_disabled()

    self.calib_vision_button.setEnabled(True)
    
    self.calib_vision_button.clicked.connect(self.set_disabled)
    self.calib_sensor_button.clicked.connect(self.set_disabled)
    self.object_grasp_button.clicked.connect(self.set_disabled)
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

    calib_layout = QVBoxLayout()
    calib_layout.addWidget(self.calib_vision_button)
    calib_layout.addWidget(self.calib_sensor_button)
    calib_layout.addWidget(self.object_grasp_button)
    calib_layout.addWidget(self.object_recog_button)

    calib_group_box = QGroupBox('System control')
    calib_group_box.setLayout(calib_layout)

    main_layout = QVBoxLayout()
    main_layout.addLayout(subject_layout)
    main_layout.addLayout(subject_buttons_layout)
    main_layout.addWidget(calib_group_box)

    self.setLayout(main_layout)
  
  def set_disabled(self):
    self.new_subject_button.setDisabled(True)
    self.terminate_subject_button.setDisabled(True)
    self.calib_vision_button.setDisabled(True)
    self.calib_sensor_button.setDisabled(True)
    self.object_grasp_button.setDisabled(True)
    self.object_recog_button.setDisabled(True)
  
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