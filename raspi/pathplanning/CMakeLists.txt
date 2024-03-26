cmake_minimum_required(VERSION 3.0)
project(Eurobot)

# Set rplidar sdk directory
set(SDK_DIR /home/bot/eurobot/raspi/eurobot-lidar/rplidar_sdk)

# Add our C++ source files
set(EUROBOTFILES 
    main.cpp
    pathplanning.cpp
)

# Include sdk
include_directories(${SDK_DIR}/sdk/include ${SDK_DIR}/sdk/src)

FILE(GLOB SDKSRC ${SDK_DIR}/sdk/src/*.cpp)
FILE(GLOB SDKARCH ${SDK_DIR}/sdk/src/arch/linux/*.cpp)
FILE(GLOB SDKHAL ${SDK_DIR}/sdk/src/hal/*.cpp)
FILE(GLOB SDKDATAUNPACKER ${SDK_DIR}/sdk/src/dataunpacker/*.cpp)
FILE(GLOB SDKUNPACKER ${SDK_DIR}/sdk/src/dataunpacker/unpacker/*.cpp)

# Build executable
add_executable(${PROJECT_NAME} ${EUROBOTFILES} ${SDKARCH} ${SDKSRC} ${SDKHAL} ${SDKUNPACKER} ${SDKDATAUNPACKER})

# Link libraries
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} Threads::Threads)
