project(doosan_xbot2_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)