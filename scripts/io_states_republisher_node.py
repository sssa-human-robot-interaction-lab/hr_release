#!/usr/bin/env python3

import rospy
from ur_msgs.msg import IOStates
from hr_release.msg import IOStatesStamped

class IOStatesRepublisher:

  def __init__(self) -> None:
    self.msg = IOStatesStamped()
    self.pub = rospy.Publisher('io_states_stamped',IOStatesStamped,queue_size=1000) 
    sub = rospy.Subscriber('ur_hardware_interface/io_states',IOStates,self.republish_cb)

  def republish_cb(self, msg:IOStates):
    self.msg.header.stamp = rospy.Time.now()
    self.msg.io_states = msg
    self.pub.publish(self.msg)

def main():
  rospy.init_node('io_states_republisher_node')
  repub = IOStatesRepublisher()
  rospy.loginfo('IOStates republisher ready!')
  rospy.spin()

if __name__ == '__main__':
  main()

