project(doosan_xbot2_hal)

find_package(ament_cmake REQUIRED)

# XBot2
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

# ros2
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")
# set manually xenomai support
set(XBOT2_ENABLE_XENO OFF)

# headers
include_directories(include ${XBOT2_INCLUDE_DIRS} )

# globalazze
add_library(global_vars SHARED src/global_drfl.cpp)
install(TARGETS global_vars
        DESTINATION lib)

# xbot2 devices
add_xbot2_device(doosan 
                 src/doosan_device.cpp)

add_xbot2_device(sander 
                 src/sander_device.cpp)

#  TBD use @Matteodellevedove find for the libraries
add_library(DRFL STATIC IMPORTED)
set_target_properties(DRFL PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/libDRFL.a")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/external/libDRFL.a"
        DESTINATION lib)

add_library(PocoFoundation SHARED IMPORTED)
set_target_properties(PocoFoundation PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoFoundation.so.62")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoFoundation.so.62"
        DESTINATION lib)

add_library(PocoNet SHARED IMPORTED)
set_target_properties(PocoNet PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoNet.so.62")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoNet.so.62"
        DESTINATION lib)


target_link_libraries(doosan
    PRIVATE 
    xbot2::xbot2
    matlogger2::matlogger2
    DRFL
    PocoFoundation
    PocoNet
    global_vars
    )

install(TARGETS doosan
        DESTINATION lib)


target_link_libraries(sander
    PRIVATE 
    xbot2::xbot2
    matlogger2::matlogger2
    DRFL
    PocoFoundation
    PocoNet
    global_vars
    )

install(TARGETS sander
        DESTINATION lib)



add_xbot2_plugin(sander_control
                src/sander_control.cpp)

target_include_directories(sander_control PRIVATE
  ${xbot2_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS})

#ament_target_dependencies(sander_control rclcpp std_srvs)

message("---------------------------------------------- ${rclcpp_LIBRARIES}")

target_link_libraries(sander_control PRIVATE
  xbot2::xbot2_ros2_support
  PUBLIC
  rclcpp::rclcpp
  ${std_srvs_LIBRARIES}
  matlogger2::matlogger2)

install(
    TARGETS sander_control
    DESTINATION lib
    )


ament_package()