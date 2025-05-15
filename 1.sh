#!/bin/sh

CONTAINER_NAME="bot"  
COMMANDS=". install/setup.bash && ros2 run send_num camera_server"  

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"