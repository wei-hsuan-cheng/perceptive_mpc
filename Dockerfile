# Select base image based on target architecture (defaults to amd64).
ARG TARGETARCH=amd64

# Native amd64 image already ships with ROS Melodic desktop full.
FROM osrf/ros:melodic-desktop-full AS amd64

# Build an arm64 ROS Melodic desktop full rootfs from Ubuntu 18.04 (bionic).
FROM arm64v8/ubuntu:bionic AS arm64
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /tmp/ros.asc \
 && apt-key add /tmp/ros.asc \
 && echo "deb [arch=arm64] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
 && rm -rf /var/lib/apt/lists/* /tmp/ros.asc
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV PATH=${ROS_ROOT}/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python2.7/dist-packages
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

FROM ${TARGETARCH}

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install catkin + wstool + glog_catkin
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    python-catkin-tools \
    python-wstool \
    libglpk-dev \
    autoconf \
    automake \
    libtool \
    pkg-config \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash

# install system deps
RUN apt update && apt install libglpk-dev -y

# Set the working directory
WORKDIR /usr/src

RUN mkdir -p perceptive_mpc_ws/src/perceptive_mpc
WORKDIR /usr/src/perceptive_mpc_ws

RUN catkin init
RUN catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
WORKDIR /usr/src/perceptive_mpc_ws/src
COPY . ./perceptive_mpc/
RUN wstool init . ./perceptive_mpc/perceptive_mpc_https.rosinstall
RUN catkin build perceptive_mpc

#source workspace
WORKDIR /usr/src/perceptive_mpc_ws
