#!/bin/bash
#xhost +

. install/setup.bash

ros2 launch youbot_moveit start.launch.py arm_planner:=PRM