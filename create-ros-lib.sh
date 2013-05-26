#!/bin/bash
cd `dirname $0`

LIBS_DIR=pinky/libs

mkdir -p $LIBS_DIR

mv -f $LIBS_DIR/ros_lib $LIBS_DIR/ros_lib.backup

cd $LIBS_DIR
rosrun rosserial_arduino make_libraries.py .
