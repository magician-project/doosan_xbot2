cmake_minimum_required(VERSION 3.14.4)

project(doosan_xbot2_hal_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY hal
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES doosan_xbot2_config.yaml 
  DESTINATION share/${PROJECT_NAME}
)

install(FILES doosan_impedance_config.yaml 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()