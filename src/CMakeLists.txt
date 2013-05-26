#
# Top-level CMake file for Pink project.
#

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../arduino-cmake/cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8)

# Uncomment these in order to be able to see the list of supported boards.
# print_board_list()
# print_programmer_list()


project(Pinky C CXX)

set(ARDUINO_DEFAULT_BOARD mega2560)
set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0)

generate_arduino_library(example_lib
    SRCS example/example.cpp
    HDRS example/example.h)
    
generate_arduino_firmware(pinky
    SRCS pinky_main.cpp
    LIBS example_lib)
