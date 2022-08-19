from time import sleep
from functools import partial

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from artificial_hands_py.artificial_hands_py_base import *

def q_confirm_dialog(parent,str) -> bool:
    dlg = QMessageBox(parent)
    dlg.setText(str)
    dlg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
    dlg.setIcon(QMessageBox.Question)
    return dlg.exec() == QMessageBox.Yes

class QResultButton(QPushButton):

  changed = pyqtSignal()

  def __init__(self,button : str, parent, result_cb):
    super().__init__(button, parent)

    self.result_cb = result_cb
    self.result = False
    self.success = False
    self.percentage = 0
    self.alpha = 50
    self.set_button_rgb([255,0,0,self.alpha])
    
  def set_result_button(self, goal, result):
    self.result = True
    self.success = result.success
    self.result_cb()
    if result.success:
      self.set_button_rgb([0,255,0,self.alpha])
    else:
      self.set_button_rgb([255,0,0,self.alpha])
    self.changed.emit()
  
  def set_progress_button(self, feedback):
    for c in range(feedback.percentage-self.percentage):
      r = int(255 - 2.55*(self.percentage + c))
      g = 255 - r
      self.set_button_rgb([r,g,0,self.alpha])
      sleep(0.05)
    self.percentage = feedback.percentage

  def set_button_rgb(self, rgba : list):
    col = QColor.fromRgb(rgba[0], rgba[1], rgba[2], rgba[3])
    if col.isValid():
      self.setStyleSheet("background-color: {}".format(col.name()))
  
  def reset(self):
    self.set_button_rgb([255,0,0,self.alpha])
    self.result = False
    self.success = False