cmake_minimum_required(VERSION 3.1.3)
project(planners_brubotics)

find_package(catkin REQUIRED)
find_package(octomap REQUIRED)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

