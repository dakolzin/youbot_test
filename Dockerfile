ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
   ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install ros package
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y\
      ros-humble-ros2-control \
      ros-humble-graph-msgs \
      ros*controller* \
      ros-humble-rviz-visual-tools \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-moveit \
      ros-humble-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /youbot

ENTRYPOINT [ "/bin/bash"]