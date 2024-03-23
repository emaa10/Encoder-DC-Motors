cmake_minimum_required(VERSION 3.0)
project(Main)

set(CMAKE_CXX_STANDARD 11)

find_library(WIRINGPI_LIB wiringPi)

if(NOT WIRINGPI_LIB)
    message(FATAL_ERROR "WiringPi library not found")
endif()

# Füge den Include-Pfad von WiringPi hinzu
include_directories(/usr/local/include)

add_executable(serial_sender serial_sender.cpp)
target_link_libraries(serial_sender ${WIRINGPI_LIB})
