#!/bin/bash

rostopic pub -1 /command megabot/DriveCommand -- '{Cmd: [{TargetSpeed: 8000}, {TargetSpeed: -1000}], TimeToLive: 20000}'
