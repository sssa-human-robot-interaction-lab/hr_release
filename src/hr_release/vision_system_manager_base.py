import numpy as np
from threading import Lock
from functools import partial

import rospy, tf, tf.transformations as ts

from geometry_msgs.msg import TransformStamped

from artificial_hands_msgs.msg import CartesianTrajectoryPointStamped
from artificial_hands_py.artificial_hands_py_base import *

@singleton
class VisionSystem:
  
  lock = Lock()

  br = tf.TransformBroadcaster()

  def __init__(self) -> None:

    def init_pnt(frame_id : str):
      pnt = CartesianTrajectoryPointStamped()
      pnt.header.frame_id = frame_id
      pnt.point.pose.orientation.w = 1
      return pnt

    self.rec_pnt = init_pnt('receiver')
    self.giv_pnt = init_pnt('giver')
    self.obj_pnt = init_pnt('object')   
    
    rec_sub = rospy.Subscriber('vicon/MagicBall/ball',TransformStamped,partial(self.vicon_cb,self.rec_pnt))
    giv_sub = rospy.Subscriber('vicon/Mia/hand',TransformStamped,partial(self.vicon_cb,self.giv_pnt))
    obj_sub = rospy.Subscriber('vicon/TestObjext/object',TransformStamped,partial(self.vicon_cb,self.obj_pnt))

    self.pnts = [self.rec_pnt,self.giv_pnt,self.obj_pnt]

    self.R = ts.rotation_matrix(0,[0,0,1])
  
    tf_tim = rospy.Timer(rospy.Duration(0.01), self.update_tf)

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

  def get_position(self, pnt : CartesianTrajectoryPointStamped) -> list:
    return [pnt.point.pose.position.x,pnt.point.pose.position.y,pnt.point.pose.position.z].copy()

  def update_tf(self, event):
    self.lock.acquire()
    for pnt in self.pnts:
      self.br.sendTransform([pnt.point.pose.position.x,pnt.point.pose.position.y,pnt.point.pose.position.z],
      quat_to_list(pnt.point.pose.orientation),rospy.Time.now(),pnt.header.frame_id,"base")
    self.lock.release()
   
  def vicon_cb(self, pnt : CartesianTrajectoryPointStamped, msg : TransformStamped):
    self.lock.acquire()
    # pnt_new = np.reshape(np.dot(self.R,np.transpose([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z,1.0])),(4,1))
    # pnt.point.pose.position.x = pnt_new[0,0]
    # pnt.point.pose.position.y = pnt_new[1,0]
    # pnt.point.pose.position.z = pnt_new[2,0]
    pnt.point.pose = matrix_to_pose(np.dot(self.R,transform_to_matrix(msg.transform)))
    self.lock.release()
