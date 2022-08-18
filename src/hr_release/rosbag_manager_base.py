from abc import ABC
from os import mkdir, remove
from time import strftime
from functools import partial

import rospy, rosbag

class ROSBagManagerBase():

  def __init__(self, bag_dict : dict) -> None:
    self.__bag_dir : str
    self.__bag_name : str
    self.__bag_record = False

    subs = []
    for topic in bag_dict.keys():
      subs.append(rospy.Subscriber(topic,bag_dict[topic],partial(self.__bag_write, topic)))
  
  def bag_write(self, msg, topic): 
    if self.__bag_record:
      self.__bag.write(topic,msg)

  def bag_mkdir(self, name : str, readme : str = None) -> str:
    self.__bag_dir = name + "_{0}".format(strftime("%Y%m%d_%H%M%S"))
    mkdir(self.__bag_dir)
    if readme is not None:
      with open(self.__bag_dir+'/README.txt','w') as f:
        f.write(readme)
    return self.__bag_dir

  def bag_open(self, name : str) -> str:
    self.__bag_name = self.__bag_dir + name
    self.__bag = rosbag.Bag(self.__bag_name,'w')
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
  
class ROSBagManager(ROSBagManagerBase,ABC):

  def __init__(self, bag_dict : dict) -> None:
    super().__init__(bag_dict)
