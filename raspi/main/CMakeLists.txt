cmake_minimum_required(VERSION 3.0)
project(Main)

set(CMAKE_CXX_STANDARD 11)

set(SDK_DIR ../eurobot-lidar/rplidar_sdk) 
include_directories(${SDK_DIR}/sdk/include ${SDK_DIR}/sdk/src)
FILE(GLOB SDKSRC ${SDK_DIR}/sdk/src/*.cpp)
FILE(GLOB SDKARCH ${SDK_DIR}/sdk/src/arch/linux/*.cpp)
FILE(GLOB SDKHAL ${SDK_DIR}/sdk/src/hal/*.cpp)
FILE(GLOB SDKDATAUNPACKER ${SDK_DIR}/sdk/src/dataunpacker/*.cpp)
FILE(GLOB SDKUNPACKER ${SDK_DIR}/sdk/src/dataunpacker/unpacker/*.cpp)
# rplidar sdk dir

set(SOURCEFILES
    main.cpp
    pathplanning.cpp
)

add_executable(main ${SOURCEFILES} ${SDKARCH} ${SDKSRC} ${SDKHAL} ${SDKUNPACKER} ${SDKDATAUNPACKER})

find_package(Threads REQUIRED)

target_link_libraries(main wiringPi Threads::Threads)

