cmake_minimum_required(VERSION 3.0)
project(Main)

set(CMAKE_CXX_STANDARD 11)

# Include directories for your project
include_directories(${CMAKE_SOURCE_DIR}/libraries/WiringPi/wiringPi)

# Add the source files of your project
add_executable(main main.cpp)

# Add WiringPi and WiringSerial libraries to your project
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries/WiringPi)

# Link WiringPi and WiringSerial libraries to your executable
target_link_libraries(main WiringPi)

