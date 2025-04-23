#!/bin/bash

xhost +local:

RANDOM_DOMAIN_ID=$(( RANDOM % 250 ))
echo "Используется ROS_DOMAIN_ID: $RANDOM_DOMAIN_ID"

docker run --name bot -it --net=host \
  --env "QT_X11_NO_MITSHM=1" \
  --env DISPLAY=unix$DISPLAY \
  --env ROS_DOMAIN_ID="$RANDOM_DOMAIN_ID" \
  --volume "$(pwd):/youbot" \
  --privileged \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  youbot \
  -c ". /opt/ros/humble/setup.bash; cd /youbot; colcon build; bash runlaunch.sh"
