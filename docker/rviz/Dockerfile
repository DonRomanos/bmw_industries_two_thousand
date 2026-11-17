# RViz2 (ROS 2 Lyrical, Ubuntu 24.04) for viewing the Jetson's nvblox/cuVSLAM output from the
# dev host. Build and run with docker/rviz/run.sh.

ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Lyrical for noble from packages.ros.org; nvblox message/RViz-plugin packages from the
# Isaac ROS repo (same release as the Jetson image).
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates gnupg \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor \
        -o /usr/share/keyrings/isaac-ros.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main" \
        > /etc/apt/sources.list.d/isaac-ros.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-rviz2 \
        ros-lyrical-tf2-tools \
        ros-lyrical-rmw-fastrtps-cpp \
        ros-lyrical-nvblox-msgs \
        ros-lyrical-nvblox-rviz-plugin \
        ros-lyrical-isaac-ros-visual-slam-interfaces \
        mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc
COPY stereo.rviz /config/stereo.rviz
CMD ["bash", "-c", "source /opt/ros/lyrical/setup.bash && exec rviz2 -d /config/stereo.rviz"]
