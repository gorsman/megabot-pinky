#
# Top-level CMake file for Pink project.
#

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../arduino-cmake/cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8)

project(Pinky C CXX)

# Uncomment these in order to be able to see the list of supported boards.
# print_board_list()
# print_programmer_list()

# set(ARDUINO_DEFAULT_BOARD mega2560)
# set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0)

set(ARDUINO_DEFAULT_BOARD uno)
set(ARDUINO_DEFAULT_PORT /dev/ttyACM0)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
set(ros_lib_RECURSE True)

generate_arduino_library(example_lib
    SRCS example/example.cpp
    HDRS example/example.h)

generate_arduino_library(motor_lib
    SRCS motor_emulator.cpp
         motor_hardware.cpp
         motor_speed_measurer.cpp
         motor_controller.cpp
    HDRS motor.h
         motor_emulator.h
         motor_hardware.h
         motor_speed_measurer.h
         motor_controller.h)
    
generate_arduino_firmware(pinky
    SRCS pinky_main.cpp
    LIBS example_lib motor_lib)
