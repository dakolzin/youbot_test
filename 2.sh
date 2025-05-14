#!/bin/sh

CONTAINER_NAME="bot"  
COMMANDS=". install/setup.bash && ros2 run send_num pose_bridge"  

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"