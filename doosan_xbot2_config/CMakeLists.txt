cmake_minimum_required(VERSION 3.0)
project(doosan_xbot2_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY hal
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES doosan_xbot2_config.yaml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
