cmake_minimum_required(VERSION 3.0)
project(Main)

set(CMAKE_CXX_STANDARD 11)

# Include directories
include_directories(
    src
    ../eurobot-lidar/rplidar_sdk/sdk/include
    ../eurobot-lidar/rplidar_sdk/sdk/src
    ../eurobot-lidar/rplidar_sdk/sdk/src/arch/linux
    ../eurobot-lidar/rplidar_sdk/sdk/src/hal
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/unpacker
)

# Source files
set(SOURCEFILES
    src/main.cpp
    src/pathplanning.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/arch/linux/serial.cc
    ../eurobot-lidar/rplidar_sdk/sdk/src/hal/thread.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/hal/event.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/data_unpack.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/data_unpack_crc.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/data_unpack_exception.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/data_unpack_impl.cpp
    ../eurobot-lidar/rplidar_sdk/sdk/src/dataunpacker/data_unpack_rplidar_protocol.cpp
)

# Add executable
add_executable(main ${SOURCEFILES})

# Find required libraries
find_package(Threads REQUIRED)

# Link libraries
target_link_libraries(main wiringPi Threads::Threads)
