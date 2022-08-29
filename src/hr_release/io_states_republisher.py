import rospy
from ur_msgs.msg import IOStates
from hr_release.msg import IOStatesStamped

from PyQt5.QtWidgets import QWidget

from artificial_hands_py.artificial_hands_py_base import singleton

@singleton
class IOStatesRepublisher:

  def __init__(self) -> None:
    self.__analog_in_th = 1.0
    self.analog_in_active = False
    self.analog_in = []
    self.analog_out = []
    self.__msg = IOStatesStamped()
    self.__pub = rospy.Publisher('io_states_stamped',IOStatesStamped,queue_size=1000) 
    sub = rospy.Subscriber('ur_hardware_interface/io_states',IOStates,self.republish_cb)

  def republish_cb(self, msg : IOStates):
    self.__msg.header.stamp = rospy.Time.now()
    self.__msg.io_states = msg
    self.__pub.publish(self.__msg)
    self.analog_out = msg.analog_out_states.copy()
    self.analog_in = msg.analog_in_states.copy()
    if self.analog_in_active == False:
      if self.analog_in[0].state > self.__analog_in_th and self.analog_in[1].state > self.__analog_in_th:
        self.analog_in_active = True

  def reset(self):
    self.analog_in_active = False
