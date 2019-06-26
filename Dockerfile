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

# Copy rosinstall files
COPY cartographer.rosinstall /catkin_ws/

# Make catkin_ws folder and pull rosinstalls in the dir
RUN mkdir /catkin_ws/src -p \
  && cd /catkin_ws \
  && wstool init --shallow src cartographer.rosinstall 


# Install GRPC
RUN cd /grpc \
    && git submodule update --init \
    && make install 

# (If) for some reason protobuf is not installed, install it from the source
RUN cd /grpc/third_party/protobuf \
  && make install

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
    libgoogle-glog-dev 

# Change working directory
WORKDIR /catkin_ws

# Install Proto3
RUN src/cartographer/scripts/install_proto3.sh

# Install Aync GRPC using script (Normal install does not work, probably due to that specific commit)
RUN src/cartographer/scripts/install_async_grpc.sh

# Make the project folder
RUN rosdep update \    
  && apt-get update -qq \
  && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean && rm -rf /var/lib/apt/lists/* \
  && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make_isolated --install --use-ninja --cmake-args -DBUILD_GRPC=ON"