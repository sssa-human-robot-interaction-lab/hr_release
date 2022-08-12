import numpy as np
from threading import Lock, Thread
from functools import partial
from queue import Queue

import rospy, tf, tf.transformations as ts

from geometry_msgs.msg import TransformStamped, Pose, Twist
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import CartesianTrajectoryPointStamped
from artificial_hands_py.artificial_hands_py_base import *

def sav_gol(q : Queue):
  s = 0
  g = [0.08333,0.05952,0.03571,0.0119,-0.0019,-0.03571,-0.05952,-0.08333]
  for c in g:
    s += c*q.get()
  return s

class VisionPoint:

  def __init__(self, frame_id : str, subject : str, callback, broadcaster) -> None:
    
    self.pnt = CartesianTrajectoryPointStamped()
    self.pnt.header.frame_id = frame_id
    self.pnt.point.pose.orientation.w = 1

    self.x = []
    self.xd = []
    self.xdd = []
    
    self.n_sav_gol = 8
    for n in range(self.n_sav_gol):
      self.x.append(Queue(maxsize = self.n_sav_gol))
      self.xd.append(Queue(maxsize = self.n_sav_gol))
      self.xdd.append(Queue(maxsize = self.n_sav_gol))

    sub = rospy.Subscriber('vicon/'+subject,TransformStamped,partial(callback,self.pnt))
    self.pub = rospy.Publisher('vicon_state/'+subject,CartesianTrajectoryPointStamped,queue_size=1000)
   
    self.br : tf. TransformBroadcaster = broadcaster

    d_thread = Thread(target=self.update_d)
    #d_thread.start
  
  def update_d(self):

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

      for (q, p) in (zip(self.x,pose_to_list(self.pnt.point.pose))):
        q.put(p)

      if self.x[0].full():
        td = []
        for (qd, q) in (zip(self.xd,self.x)):
          sd = sav_gol(q)
          qd.put(sd)
          td.append(sd)

      if self.xd[0].full():
        tdd = []
        for (qdd, qd) in (zip(self.xdd,self.xd)):
          sdd = sav_gol(qd)
          qdd.put(sdd)
          tdd.append(sdd)

      self.pnt.point.twist = list_to_twist(td)
      self.pnt.point.acceleration = list_to_twist(tdd)
      rate.sleep() 

  def update(self):
    self.pub.publish(self.pnt)
    self.br.sendTransform([self.pnt.point.pose.position.x,self.pnt.point.pose.position.y,self.pnt.point.pose.position.z],
      quat_to_list(self.pnt.point.pose.orientation),rospy.Time.now(),self.pnt.header.frame_id,"base")

  def get_position(self):
    return [self.pnt.point.pose.position.x,self.pnt.point.pose.position.y,self.pnt.point.pose.position.z].copy()
  
  def get_acceleration_norm(self):
    return pow(pow(self.pnt.point.acceleration.linear.x,2)+pow(self.pnt.point.acceleration.linear.y,2)+pow(self.pnt.point.acceleration.linear.z,2),.5)

  def get_pose(self):
    p = self.pnt.point.pose
    return p

@singleton
class VisionSystem:
  
  lock = Lock()

  br = tf.TransformBroadcaster()

  def __init__(self) -> None:

    self.R = ts.rotation_matrix(0,[0,0,1])

    self.rec_pnt = VisionPoint('receiver','MagicBall/ball',self.vision_cb, self.br)
    self.giv_pnt = VisionPoint('giver','Mia/hand',self.vision_cb, self.br)
    self.obj_pnt = VisionPoint('object','TestObject/object',self.vision_cb, self.br)

    self.pnts = [self.rec_pnt,self.giv_pnt,self.obj_pnt]

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
    pnt.header.stamp = msg.header.stamp
    pnt.point.pose : Pose = matrix_to_pose(np.dot(self.R,transform_to_matrix(msg.transform)))
    self.lock.release()
