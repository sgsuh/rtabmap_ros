# Development image for building & running THIS rtabmap_ros checkout on ROS 2 Jazzy.
#
# The base image already ships the rtabmap *core* library (SLAM engine, GUI libs)
# prebuilt for Ubuntu 24.04 (Jazzy's platform), so we only need to resolve and
# install the ROS dependencies of the packages in this repo. The source itself is
# bind-mounted at runtime (see docker-compose.yml) and built with colcon inside the
# container, which keeps edit -> build iterations fast on the feature branch.
FROM introlab3it/rtabmap:noble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash", "-c"]

WORKDIR /root/ros2_ws

# Copy the source only to let rosdep discover and install apt dependencies.
# This layer is cached until a package.xml changes. The copy is removed afterwards
# because the real source is bind-mounted at /root/ros2_ws/src/rtabmap_ros at runtime.
COPY . src/rtabmap_ros/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    (rosdep init 2>/dev/null || true) && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys="rtabmap grid_map_ros" && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    rm -rf src/rtabmap_ros

# Runtime extras not pulled by rosdep:
#  - image-transport-plugins: decode compressed / compressedDepth bag topics
#    (e.g. the robot_mapping_demo dataset).
#  - rviz2: the demo launch files can start RViz (rviz:=true).
RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-rviz2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# The repo is bind-mounted (owned by the host user), so let git operate on it as root.
RUN git config --global --add safe.directory /root/ros2_ws/src/rtabmap_ros

# Auto-source ROS and, when present, the workspace overlay for interactive shells.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["bash"]
