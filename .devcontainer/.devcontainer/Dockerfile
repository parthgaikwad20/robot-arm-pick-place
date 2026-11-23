FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-pytest \
        ros-humble-robot-state-publisher ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# RViz renders in software inside the browser desktop
ENV LIBGL_ALWAYS_SOFTWARE=1
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
 && echo "[ -f /workspaces/install/setup.bash ] && source /workspaces/install/setup.bash" >> /etc/bash.bashrc
