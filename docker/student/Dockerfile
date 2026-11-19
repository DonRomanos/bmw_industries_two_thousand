# Student development image: ROS 2 Lyrical + everything needed to talk to the car over Wi-Fi.
# Used by .devcontainer/devcontainer.json (build context = repository root).
#
# Contains the car's message package (car_interfaces, built into /opt/bitt_ws) and the Nav2 /
# vision message packages, but no Isaac ROS: the heavy perception runs on the Jetson.

ARG BASE_IMAGE=ros:lyrical-ros-base
FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-rmw-fastrtps-cpp \
        ros-lyrical-nav2-msgs \
        ros-lyrical-nav2-simple-commander \
        ros-lyrical-vision-msgs \
        ros-lyrical-cv-bridge \
        ros-lyrical-image-transport \
        ros-lyrical-tf2-ros-py \
        ros-lyrical-tf2-tools \
        ros-lyrical-ackermann-msgs \
        ros-lyrical-diagnostic-msgs \
        ros-lyrical-rosbag2 \
        ros-lyrical-rosbag2-py \
        ros-lyrical-rosbag2-storage-mcap \
        ros-lyrical-rviz2 \
        ros-lyrical-compressed-image-transport \
        ros-lyrical-ros-gz \
        ros-lyrical-navigation2 \
        ros-lyrical-nav2-bringup \
        ros-lyrical-nav2-mppi-controller \
        ros-lyrical-nav2-smac-planner \
        ros-lyrical-nav2-rotation-shim-controller \
        ros-lyrical-nav2-velocity-smoother \
        ros-lyrical-nav2-collision-monitor \
        ros-lyrical-nav2-rviz-plugins \
        ros-lyrical-robot-localization \
        ros-lyrical-robot-state-publisher \
        ros-lyrical-joint-state-publisher \
        ros-lyrical-xacro \
        ros-lyrical-twist-mux \
        ros-lyrical-teleop-twist-keyboard \
        ros-lyrical-pointcloud-to-laserscan \
        ros-lyrical-rtabmap-ros \
        ros-lyrical-image-proc \
        ros-lyrical-stereo-image-proc \
        ros-lyrical-depth-image-proc \
        ros-lyrical-foxglove-bridge \
        mesa-utils mesa-utils-bin libgl1-mesa-dri libegl1 libglx-mesa0 libxcb-cursor0 \
        mesa-vulkan-drivers libvulkan1 vulkan-tools xterm \
        python3-colcon-common-extensions \
        python3-opencv python3-numpy python3-pip python3-pyqt6 \
        git sudo nano iputils-ping iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Message contract with the ESP32 (lights, steering, drive mode, ...). Kept as its own underlay so
# students' workspaces rebuild without touching it.
COPY ros_ws/src/car_interfaces /opt/bitt_ws/src/car_interfaces
RUN bash -c "source /opt/ros/lyrical/setup.bash && cd /opt/bitt_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && rm -rf /opt/bitt_ws/build /opt/bitt_ws/log

# Gazebo simulation of the car (ros2 launch kitt_sim sim.launch.py): the real car's Nav2 config
# with simulated ESP32, odometry and cameras. Scenarios are mounted at runtime (scenarios/).
COPY docker/student/sim/kitt_sim /opt/kitt_sim_ws/src/kitt_sim
RUN bash -c "source /opt/ros/lyrical/setup.bash && source /opt/bitt_ws/install/setup.bash \
        && cd /opt/kitt_sim_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
    && rm -rf /opt/kitt_sim_ws/build /opt/kitt_sim_ws/log

# The car's ROS graph is reached through unicast discovery (Wi-Fi multicast is unreliable); the
# devcontainer sets ROS_STATIC_PEERS to the car's address.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Passwordless sudo for the default user so students can apt-get install what they need.
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && chmod 0440 /etc/sudoers.d/ubuntu

# Source the ROS workspaces at the very top of .bashrc: Ubuntu's stock .bashrc returns early for
# non-interactive shells (VS Code tasks, `docker exec ... bash -ic`), which would skip lines appended below it.
COPY docker/student/gpu-env.sh /etc/kitt/gpu-env.sh
RUN printf '%s\n' \
        'source /etc/kitt/gpu-env.sh' \
        'source /opt/ros/lyrical/setup.bash' \
        'source /opt/bitt_ws/install/setup.bash' \
        'source /opt/kitt_sim_ws/install/setup.bash' \
        '[ -f "$BITT_STUDENT_WS/install/setup.bash" ] && source "$BITT_STUDENT_WS/install/setup.bash"' \
        | cat - /home/ubuntu/.bashrc > /tmp/bashrc && mv /tmp/bashrc /home/ubuntu/.bashrc \
    && chown ubuntu:ubuntu /home/ubuntu/.bashrc

USER ubuntu
CMD ["sleep", "infinity"]
