cmake_minimum_required(VERSION 2.8.3)
project(pr2_fingertip_sensors)

find_package(catkin)
catkin_package()

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
