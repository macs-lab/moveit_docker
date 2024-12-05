ARG ROS_DISTRO=humble

# Base image created from ROS official image
# From Docker hub, we modify this image to add our own dependencies
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install cli tools via apt
RUN apt-get update \
    && apt-get install -y neovim python3-pip\
    && rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=noninteractive

# Need apt-get update before installing anything
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install MoveIt2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit

    # Install UR
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur

ARG DEBIAN_FRONTEND=dialog
# Create base Colcon workspace with external dependencies
# These workspaces are created as /root (default user)
RUN mkdir -p /base_ws/src
# Like cd /base_ws/src
WORKDIR /base_ws/src
# COPY copies file from this location, to the current directory (denoted with .)
COPY base_dependencies.repos .
#  Import the repositories vcs: command line tool to parse the .repos file as Python dict
RUN vcs import < base_dependencies.repos

WORKDIR /base_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Rather than in parallel building multiple packages at once, build them sequentially, minimize running out of memory

# RUN apt-get update \
#     && apt-get install -y ros-humble-ur

FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src
COPY overlay_dependencies.repos .
RUN vcs import < overlay_dependencies.repos
COPY inspection_robot_service .

WORKDIR /overlay_ws
RUN source /base_ws/install/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
COPY ./bashrc /root/.bashrc
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["bash"]

# Devel image
FROM base AS devel

# Create a different overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src

WORKDIR /overlay_ws
RUN source /base_ws/install/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# For continuous changes, add colcon build in devel_entrypoint.sh
# Set up the entrypoint
COPY ./docker/devel_entrypoint.sh /
COPY ./bashrc /root/.bashrc
ENTRYPOINT [ "/devel_entrypoint.sh" ]

CMD ["bash"]