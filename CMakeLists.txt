cmake_minimum_required(VERSION 3.14.4)
cmake_policy(SET CMP0057 NEW)

project(doosan_xbot2 VERSION 1.0.0)

# std c++17 flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(COMPILE_DOOSAN_HAL ON CACHE BOOL "Compile Doosan HAL support for XBot2" FORCE)
set(COMPILE_EXAMPLES ON CACHE BOOL "Compile XBot2 plugin examples" FORCE)

# XBot2
find_package(xbot2 REQUIRED)

# headers
include_directories(include ${XBOT2_INCLUDE_DIRS} )

# config
add_subdirectory(doosan_xbot2_hal_config)

# HAL
if(${COMPILE_DOOSAN_HAL})
    add_subdirectory(doosan_xbot2_hal)  
endif()

# examples
if(${COMPILE_EXAMPLES})
    add_subdirectory(doosan_xbot2_examples)  
endif()
