#!/bin/bash
#xhost + 

. install/setup.bash

ros2 launch youbot_moveit demo.launch.py \
  rviz_config:=$(ros2 pkg prefix youbot_description)/share/youbot_description/config/rviz.rviz



