#!/bin/sh

CONTAINER_NAME="bot"  
#COMMANDS="source /opt/ros/humble/setup.bash && colcon build && . install/setup.bash"  

COMMANDS="source /opt/ros/humble/setup.bash && . install/setup.bash" 

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"