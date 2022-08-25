import rospy
from ur_msgs.msg import IOStates
from hr_release.msg import IOStatesStamped

from PyQt5.QtWidgets import QWidget

from artificial_hands_py.artificial_hands_py_base import singleton

@singleton
class IOStatesRepublisher:

  def __init__(self) -> None:
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
