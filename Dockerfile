# Start from ros melodic robot version
FROM ros:melodic-robot

# General upgrade 
RUN apt-get update && apt-get upgrade -y

# Install essentials
RUN apt-get update -qq \
  && apt-get install -y sudo python-wstool python-rosdep ninja-build autoconf \
  && apt-get install -y build-essential autoconf libtool pkg-config \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

# Copy rosinstall files
COPY cartographer.rosinstall /catkin_ws/

# This is for recent ros keychange
RUN apt-key del -y sudo 421C365BD9FF1F717815A3895523BAEEB01FA116 \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

# Make catkin_ws folder and pull rosinstalls in the dir
RUN mkdir /catkin_ws/src -p \
  && cd /catkin_ws \
  && wstool init --shallow src cartographer.rosinstall 

# Install GRPC
RUN cd /grpc \
  && git submodule update --init \
  && make \
  && make install 

# (If) for some reason protobuf is not installed, install it form source
RUN cd /grpc/third_party/protobuf \
  && make install

# Change working directory
WORKDIR /catkin_ws

# Get dependancies, install catkin_ws
RUN rosdep update \    
  && apt-get update -qq \
  && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean && rm -rf /var/lib/apt/lists/* \
  && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make_isolated --install --install-space=/opt/ros/${ROS_DISTRO} --use-ninja --cmake-args -DBUILD_GRPC=ON" \
  && rm -rf /catkin_ws