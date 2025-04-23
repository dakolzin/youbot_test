#!/bin/sh

CONTAINER_NAME="bot"  
COMMANDS=". install/setup.bash"  

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"