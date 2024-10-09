find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

include_directories(${matlogger2_INCLUDE_DIRS})

option(ENABLE_XENO OFF "Compile against xenomai")

add_xbot2_plugin(periodic_motion
periodic_motion.cpp)

target_link_libraries(periodic_motion PRIVATE matlogger2::matlogger2)

install(
    TARGETS periodic_motion
    DESTINATION lib
    )

