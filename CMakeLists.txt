cmake_minimum_required(VERSION 2.8.3)
project(pr2_fingertip_sensors)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs sensor_msgs)

catkin_python_setup()

add_message_files(
  FILES
  PR2FingertipSensor.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
  )

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
