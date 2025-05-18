#!/bin/sh

CONTAINER_NAME="bot"  
COMMANDS="source /opt/ros/humble/setup.bash && colcon build && . install/setup.bash && ros2 run send_num bridge"  

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"