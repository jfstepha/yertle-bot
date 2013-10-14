cmake_minimum_required(VERSION 2.4.6)
project(yertle)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs differential_drive openni_launch ros_arduino_messages
)

catkin_python_setup()

install(PROGRAMS
  scripts/int2range.py
  scripts/joy_teleop.py
  scripts/laptop_bat.py
  scripts/laptop_wifi.py
  nodes/arduino_driver.py  
  nodes/arduino_node.py  
  nodes/arduino_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

