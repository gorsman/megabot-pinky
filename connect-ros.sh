#!/bin/bash
#
# This script runs 'rosserial' ROS node that connect Arduino to the ROS stack.

PORT=/dev/ttyACM0
BAUD=115200
# BAUD=9600

rosrun rosserial_python serial_node.py _port:=$PORT _baud:=$BAUD
