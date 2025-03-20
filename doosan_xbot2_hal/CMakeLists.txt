project(doosan_xbot2_hal)

find_package(ament_cmake REQUIRED)

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
    DRFL
    PocoFoundation
    PocoNet
    )

install(TARGETS doosan
        DESTINATION lib)


ament_package()