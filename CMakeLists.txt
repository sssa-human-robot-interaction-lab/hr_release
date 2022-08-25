cmake_minimum_required(VERSION 3.0.2)
project(hr_release)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  ur_msgs
  geometry_msgs
  cartesian_control_msgs
  trajectory_msgs
  message_generation
  actionlib_msgs
  dynamic_reconfigure
)

catkin_python_setup()

add_message_files(
  FILES
  IOStatesStamped.msg
)

add_action_files(
  FILES
  ForceTorqueSensorCalibration.action
  ObjectGrasp.action
  ObjectRecognition.action
  RobotHumanHandoverReaching.action
  VisionSystemCalibration.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ur_msgs
  geometry_msgs
  cartesian_control_msgs
  trajectory_msgs
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS artificial_hands_py
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hr_execution_modules_node.py
  scripts/hr_fake_vision_system_node.py
  scripts/hr_release_gui_node.py
  scripts/hr_release_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
