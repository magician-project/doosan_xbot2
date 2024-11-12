project(doosan_xbot2_examples)

# catkinized
find_package(catkin REQUIRED)
catkin_package()

find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

option(XBOT2_ENABLE_XENO ON "Compile against xenomai")
# set manually xenomai support
set(XBOT2_ENABLE_XENO ON)

include_directories(${matlogger2_INCLUDE_DIRS})



add_xbot2_plugin(periodic_motion
periodic_motion.cpp)

target_link_libraries(periodic_motion PRIVATE matlogger2::matlogger2)

install(
    TARGETS periodic_motion
    DESTINATION lib
    )

