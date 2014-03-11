#!/bin/bash

rostopic pub -1 /command megabot/DriveCommand -- '{Cmd: [{TargetSpeed: 8000}, {TargetSpeed: -1000}], TimeToLive: 10000}'
sleep 2
 
rostopic pub -1 /command megabot/DriveCommand -- '{Cmd: [{TargetSpeed: 1000, PreserveOdometry: true}, {TargetSpeed: -1000}], TimeToLive: 20000}'
