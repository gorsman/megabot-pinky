#!/bin/bash

POWER=150
rostopic pub --rate=20 /rmotor std_msgs/Float32 -- $POWER
