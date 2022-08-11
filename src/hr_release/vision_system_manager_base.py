import numpy as np
from threading import Lock
from functools import partial

import rospy, tf, tf.transformations as ts

from geometry_msgs.msg import TransformStamped, Pose

from artificial_hands_msgs.msg import CartesianTrajectoryPointStamped
from artificial_hands_py.artificial_hands_py_base import *
\
class VisionPoint:

  def __init__(self, frame_id : str, subject : str, callback, broadcaster) -> None:
    
    self.pnt = CartesianTrajectoryPointStamped()
    self.pnt.header.frame_id = frame_id
    self.pnt.point.pose.orientation.w = 1
   
    sub = rospy.Subscriber('vicon/'+subject,TransformStamped,partial(callback,self.pnt))
    self.pub = rospy.Publisher('vicon_state'+subject,CartesianTrajectoryPointStamped,queue_size=1000)
   
    self.br : tf. TransformBroadcaster = broadcaster

  def update(self):
    self.pub.publish(self.pnt)
    self.br.sendTransform([self.pnt.point.pose.position.x,self.pnt.point.pose.position.y,self.pnt.point.pose.position.z],
      quat_to_list(self.pnt.point.pose.orientation),rospy.Time.now(),self.pnt.header.frame_id,"base")

  def get_position(self):
      return [self.pnt.point.pose.position.x,self.pnt.point.pose.position.y,self.pnt.point.pose.position.z].copy()

  def get_pose(self):
      return self.pnt.point.pose

@singleton
class VisionSystem:
  
  lock = Lock()

  br = tf.TransformBroadcaster()

  def __init__(self) -> None:

    # def init_pnt(frame_id : str):
    #   pnt = CartesianTrajectoryPointStamped()
    #   pnt.header.frame_id = frame_id
    #   pnt.point.pose.orientation.w = 1
    #   return pnt

    # self.rec_pnt = init_pnt('receiver')
    # self.giv_pnt = init_pnt('giver')
    # self.obj_pnt = init_pnt('object')   
    
    # rec_sub = rospy.Subscriber('vicon/MagicBall/ball',TransformStamped,partial(self.vision_cb,self.rec_pnt))
    # giv_sub = rospy.Subscriber('vicon/Mia/hand',TransformStamped,partial(self.vision_cb,self.giv_pnt))
    # obj_sub = rospy.Subscriber('vicon/TestObject/object',TransformStamped,partial(self.vision_cb,self.obj_pnt))

    # self.rec_pub = rospy.Publisher('vicon_state/MagicBall/ball',CartesianTrajectoryPointStamped,queue_size=1000)
    # self.giv_pub = rospy.Publisher('vicon_State/Mia/hand',CartesianTrajectoryPointStamped,queue_size=1000)
    # self.obj_pub = rospy.Publisher('vicon_state/TestObject/object',CartesianTrajectoryPointStamped,queue_size=1000)

    self.rec_pnt = VisionPoint('receiver','MagicBall/ball',self.vision_cb, self.br)
    self.giv_pnt = VisionPoint('giver','Mia/hand',self.vision_cb, self.br)
    self.obj_pnt = VisionPoint('object','TestObject/object',self.vision_cb, self.br)

    self.pnts = [self.rec_pnt,self.giv_pnt,self.obj_pnt]

    self.R = ts.rotation_matrix(0,[0,0,1])
  
    tf_tim = rospy.Timer(rospy.Duration(0.01), self.update_tf)

  def reset_calibration_matrix(self):
    self.R = ts.rotation_matrix(0,[0,0,1])

  def four_point_calibration(self, pnts : list, arm_pnt : list):

    self.lock.acquire()

    p0 = np.array(pnts[0])
    p1 = np.array(pnts[1])
    p2 = np.array(pnts[2])
    
    u = p1 - p0
    v = p2 - p1
    w = np.cross(u,v)
    
    z_ver = w/np.linalg.norm(w)
    y_ver = np.cross(w,u)/np.linalg.norm(np.cross(w,u))
    x_ver = u/np.linalg.norm(u)

    R = np.matrix([x_ver,y_ver,z_ver])

    T = np.ndarray((4,4))
    T[0:3,0:3] = R
    T[0:3,3] = np.dot(R,np.transpose(-p0)) + np.array(arm_pnt[0:3])

    self.R = T

    self.lock.release()

  def update_tf(self, event):
    self.lock.acquire()
    for pnt in self.pnts:
      pnt.update()
    self.lock.release()
   
  def vision_cb(self, pnt : CartesianTrajectoryPointStamped, msg : TransformStamped):
    self.lock.acquire()
    dt = msg.header.stamp.to_sec() - pnt.header.stamp.to_sec()
    c_point = pnt.point
    pnt.header.stamp = msg.header.stamp
    pnt.point.pose : Pose = matrix_to_pose(np.dot(self.R,transform_to_matrix(msg.transform)))
    pnt.point.twist.linear.x = (pnt.point.pose.position.x - c_point.pose.position.x)/dt
    pnt.point.twist.linear.y = (pnt.point.pose.position.y - c_point.pose.position.y)/dt
    pnt.point.twist.linear.z = (pnt.point.pose.position.z - c_point.pose.position.z)/dt
    pnt.point.acceleration.linear.x = (pnt.point.twist.linear.x - c_point.twist.linear.x)/dt
    pnt.point.acceleration.linear.y = (pnt.point.twist.linear.y - c_point.twist.linear.y)/dt
    pnt.point.acceleration.linear.z = (pnt.point.twist.linear.z - c_point.twist.linear.z)/dt
    self.lock.release()
