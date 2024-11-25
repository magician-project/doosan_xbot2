project(doosan_xbot2_hal)

find_package(catkin REQUIRED)
catkin_package()

# XBot2
find_package(xbot2 REQUIRED)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")
# set manually xenomai support
set(XBOT2_ENABLE_XENO OFF)

# headers
include_directories(include ${XBOT2_INCLUDE_DIRS} )

# xbot2 device
add_xbot2_device(doosan 
                 src/doosan_device.cpp)

target_link_libraries(doosan
    PRIVATE 
    xbot2::xbot2
    ${CMAKE_CURRENT_SOURCE_DIR}/external/libDRFL.a
    ${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoFoundation.so.62
    ${CMAKE_CURRENT_SOURCE_DIR}/external/libPocoNet.so.62
    )

install(TARGETS doosan
        DESTINATION lib)


