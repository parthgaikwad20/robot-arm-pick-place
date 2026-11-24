FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# The image already contains RViz2, robot_state_publisher and colcon.
# Drop the ROS apt repo (its signing key can be expired in older images)
# so that apt-get keeps working for us and for the desktop feature.
RUN rm -f /etc/apt/sources.list.d/ros* \
 && apt-get update \
 && apt-get install -y --no-install-recommends python3-pytest \
 && rm -rf /var/lib/apt/lists/*

# RViz renders in software inside the browser desktop
ENV LIBGL_ALWAYS_SOFTWARE=1
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
 && echo 'for f in /workspaces/*/install/setup.bash; do [ -f "$f" ] && source "$f"; done' >> /etc/bash.bashrc
