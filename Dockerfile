# Start from ros melodic version 
FROM ros:melodic

# General upgrade 
RUN apt-get update && apt-get upgrade -y

# This is for recent ros keychange
RUN apt-key del -y sudo 421C365BD9FF1F717815A3895523BAEEB01FA116 \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

# Install essentials
RUN apt-get update -qq \
  && apt-get install -y sudo python-wstool python-rosdep ninja-build autoconf \
  && apt-get install -y build-essential autoconf libtool pkg-config \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install some remaining CMakeDebs for Asyc GRPC (As per install_deb script)
RUN apt-get update -y \
    && apt-get install -y \
    libgtest-dev \
    clang \
    libc++-dev \
    cmake \
    git \
    google-mock \
    libgflags-dev \
    libgoogle-glog-dev\
    libeigen3-dev
 
# First, we invalidate the entire cache if  has
# changed. This file's content changes whenever master changes. See:
# http://stackoverflow.com/questions/36996046/how-to-prevent-dockerfile-caching-git-clone

# This image depends on shreyasgokhale/cartographer_ros

ADD https://api.github.com/repos/shreyasgokhale/cartographer_ros/git/refs/heads/master master.json

# Copy rosinstall files
COPY custom_cartographer.rosinstall /catkin_ws/

# Make catkin_ws folder and pull rosinstalls in the dir
RUN mkdir /catkin_ws/src -p \
  && cd /catkin_ws \
  && wstool init --shallow src custom_cartographer.rosinstall 

# Install GRPC
RUN cd /grpc \
    && git submodule update --init \
    && make install

# (If) for some reason protobuf is not installed, install it from the source
RUN cd /grpc/third_party/protobuf \
  && make install

# Change working directory
WORKDIR /catkin_ws

# Install Proto3
RUN src/cartographer/scripts/install_proto3.sh

# Install Aync GRPC using script (Normal install does not work, probably due to that specific commit)
RUN src/cartographer/scripts/install_async_grpc.sh

# Update apts for rosdep update
RUN apt-get update -y

# Copy project scripts
COPY scripts/install_deps.sh /install_deps.sh 
COPY scripts/compile_project.sh /compile_project.sh 
RUN chmod +x /compile_project.sh /install_deps.sh

# Compile Project
RUN /install_deps.sh "/catkin_ws"
RUN /compile_project.sh "/catkin_ws"

# Copy additional files
COPY postinstall/agent_cartographer.launch /catkin_ws/src/cartographer_ros/launch/agent_cartographer.launch
COPY postinstall/agent_server.lua /catkin_ws/src/cartographer_ros/configuration_files/agent_server.lua 
COPY postinstall/agent_node.lua /catkin_ws/src/cartographer_ros/configuration_files/agent_node.lua
COPY postinstall/cloud_server.lua /catkin_ws/src/cartographer_ros/configuration_files/cloud_server.lua 

# Copy start script
COPY scripts/start_server.sh /start_server.sh 

# Start server script
RUN chmod +x /start_server.sh "/catkin_ws"
