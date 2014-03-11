#!/bin/bash

SPEED=-1000
TIME=5000
rostopic pub -1 /command megabot/DriveCommand -- "{Cmd: [{TargetSpeed: $SPEED}, {TargetSpeed: $SPEED}], TimeToLive: $TIME}"
exit
rostopic pub -1 /command megabot/DriveCommand -- '{Cmd: [{TargetSpeed: 4000}, {TargetSpeed: -1000}], TimeToLive: 10000}'
sleep 2
 
rostopic pub -1 /command megabot/DriveCommand -- '{Cmd: [{TargetSpeed: 1000, PreserveOdometry: true}, {TargetSpeed: -1000}], TimeToLive: 20000}'
