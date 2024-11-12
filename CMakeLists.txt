cmake_minimum_required(VERSION 3.0)

project(doosan_xbot2 VERSION 1.0.0)

# std c++17 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# XBot2
find_package(xbot2 REQUIRED)

# headers
include_directories(include ${XBOT2_INCLUDE_DIRS} )

# config
add_subdirectory(doosan_xbot2_config)

# gazebo
add_subdirectory(doosan_xbot2_gazebo)

# HAL
option(COMPILE_DOOSAN_HAL ON "Compile Doosan HAL support for XBot2")
if(${COMPILE_DOOSAN_HAL})
    add_subdirectory(doosan_xbot2_hal)  
endif()

# examples
option(COMPILE_EXAMPLES ON "Compile XBot2 plugin examples")
if(${COMPILE_EXAMPLES})
    add_subdirectory(doosan_xbot2_examples)  
endif()
