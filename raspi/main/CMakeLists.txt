cmake_minimum_required(VERSION 3.0)
project(Main)

set(CMAKE_CXX_STANDARD 11)

# Include directories for your project
include_directories(${CMAKE_SOURCE_DIR}/libraries/WiringPi/wiringPi)

# Add the source files of your project
add_executable(main main.cpp)

# Add WiringPi source files to your project
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries/WiringPi/wiringPi)

# Link WiringPi library to your executable
target_link_libraries(main wiringPi)

