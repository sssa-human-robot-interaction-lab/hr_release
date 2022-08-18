#!/usr/bin/env python3

import rospy, tf2_ros

from geometry_msgs.msg import TransformStamped

def main():

  rospy.init_node('fake_vision_system_node')

  tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
  tf_listener = tf2_ros.TransformListener(tf_buffer)

  rospy.sleep(1)

  giv_pub = rospy.Publisher('vicon/Mia/hand',TransformStamped,queue_size=1000)
  obj_pub = rospy.Publisher('vicon/TestObject/object',TransformStamped,queue_size=1000)
  
  obj_to_base = TransformStamped()
  obj_to_base.header.frame_id = 'object'
  obj_to_base.transform.translation.x = -0.3
  obj_to_base.transform.translation.y = 0.5
  obj_to_base.transform.translation.z = 0.0
  obj_to_base.transform.rotation.x = 0.99
  obj_to_base.transform.rotation.y = 0.12
  obj_to_base.transform.rotation.z = -0.01
  obj_to_base.transform.rotation.w = -0.04

  rate = rospy.Rate(50)

  rospy.loginfo('Fake vision system ready!')

  while not rospy.is_shutdown():

    obj_to_base.header.stamp = rospy.Time.now()

    giv_to_base : TransformStamped
    giv_to_base = tf_buffer.lookup_transform('base','probe',rospy.Time(0))
    giv_to_base.header.frame_id = 'hand'

    obj_pub.publish(obj_to_base)
    giv_pub.publish(giv_to_base)

    rate.sleep()

if __name__ == '__main__':
  main()