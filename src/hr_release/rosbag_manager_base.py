from os import mkdir, remove
from time import strftime
from functools import partial
from threading import Lock

from PyQt5.QtWidgets import QWidget

import rospy, rosbag

from artificial_hands_py.artificial_hands_py_base import singleton

@singleton
class ROSBagManagerBase:
  __lock = Lock()

  def __init__(self) -> None:
    self.__bag_dir : str
    self.__bag_name : str
    self.__bag_record = False

  def bag_create(self, bag_dict : dict):
    subs = []
    for topic in bag_dict.keys():
      subs.append(rospy.Subscriber(topic,bag_dict[topic],partial(self.bag_write, topic)))
  
  def bag_write(self, msg, topic):
    if self.__bag_record:
      self.__lock.acquire()
      self.__bag.write(topic,msg)
      self.__lock.release()

  def bag_mkdir(self, name : str, readme : str = None) -> str:
    self.__bag_dir = name + "_{0}".format(strftime("%Y%m%d_%H%M%S"))
    mkdir(self.__bag_dir)
    if readme is not None:
      with open(self.__bag_dir+'/README.txt','w') as f:
        f.write(readme)
    return self.__bag_dir

  def bag_open(self, name : str) -> str:
    self.__bag_name = self.__bag_dir + "/{}.bag".format(name)
    self.__bag = rosbag.Bag(self.__bag_name,'w')
    rospy.sleep(0.5)
    self.__bag_record = True
    return self.__bag_name

  def bag_close(self):
    self.__bag_record = False
    self.__bag.close()

  def bag_remove(self, name : str = None):
    if name is None:
      remove(self.__bag_name)
      return
    remove(name)

class QROSBagManager(QWidget):

  def __init__(self) -> None:
    super().__init__()    
    
    self.bag = ROSBagManagerBase()
