FROM nvidia/cudagl:9.0-devel-ubuntu16.04

# Setup your sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Installation

RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init
RUN rosdep update


RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential wget\
    && rm -rf /var/lib/apt/lists/*


#--------------------
# Setup nvidia-docker2
#--------------------
# Reference: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#--------------------
# Install OpenAI gym
#--------------------

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip && \
    apt-get clean && \
rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

RUN pip install gym

#--------------------
# Install gym-gazebo
#--------------------

RUN git clone https://github.com/d3637042/gym-gazebo.git /root/gym-gazebo
WORKDIR /root/gym-gazebo
RUN pip install -e .


#--------------------
# Install gzweb
#------------------

RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get update && apt-get install -y \
    imagemagick \
    nodejs \
    libjansson-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /root
RUN hg clone https://bitbucket.org/osrf/gzweb
WORKDIR /root/gzweb
RUN hg up gzweb_1.4.0
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN npm run deploy -m

#--------------------
# Install Gym-Gazebo Turtlebot dependencies
#------------------

RUN apt-get update && apt-get install -y \
    ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-turtlebot-create \
    ros-kinetic-yujin-ocs \
    ros-kinetic-yocs-msgs \
    ros-kinetic-xacro \
    ros-kinetic-roslint \
    ros-kinetic-ros-control \
    ros-kinetic-image-common \
    ros-kinetic-control-toolbox \
    ros-kinetic-roslint \
    ros-kinetic-driver-common \
    ros-kinetic-navigation \
    ros-kinetic-joystick-drivers \
    ros-kinetic-kobuki-desktop \
    ros-kinetic-kobuki-core \
    ros-kinetic-kobuki \
    ros-kinetic-kobuki-msgs \
    ros-kinetic-ecl \
    ros-kinetic-pcl-ros \
    ros-kinetic-pointcloud-to-laserscan \
    libbullet-dev \
    libusb-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    python-skimage \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN apt remove '.*gazebo.*' '.*sdformat.*' '.*ignition-.*' -y

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get install -y ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8*

RUN apt-get install -y psmisc vim byobu

RUN pip install keras

RUN pip install --ignore-installed enum34

RUN pip install tensorflow

RUN CUDNN_DOWNLOAD_SUM=1a3e076447d5b9860c73d9bebe7087ffcb7b0c8814fd1e506096435a2ad9ab0e && curl -fsSL http://developer.download.nvidia.com/compute/redist/cudnn/v7.0.5/cudnn-9.0-linux-x64-v7.tgz -O && echo "$CUDNN_DOWNLOAD_SUM cudnn-9.0-linux-x64-v7.tgz" | sha256sum -c - && tar --no-same-owner -xzf cudnn-9.0-linux-x64-v7.tgz -C /usr/local --wildcards 'cuda/lib64/libcudnn.so.*' && rm cudnn-9.0-linux-x64-v7.tgz && ldconfig

#--------------------
# Install Gym-Gazebo Turtlebot Example
#------------------

RUN mkdir -p /root/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src
WORKDIR /root/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src
RUN wstool init
RUN wstool merge https://gist.githubusercontent.com/AustinDeric/3362c86437d88347807beaba3501602e/raw/ecadfddc98a1865dfe60b29d17ff03a513b39279/gym-gazebo.rosinstall
RUN wstool update
WORKDIR /root/gym-gazebo/gym_gazebo/envs/installation/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
RUN echo "source /root/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world" >> ~/.bashrc
RUN export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world
RUN echo "export GAZEBO_MODEL_PATH=/root/gym-gazebo/gym_gazebo/envs/assets/models" >> ~/.bashrc


RUN echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> /root/.bashrc

WORKDIR /root/

RUN git clone https://github.com/RobotX-NCTU/robotx_gazebo robotx_gazebo

WORKDIR /root/robotx_gazebo/catkin_ws/src

RUN git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git

RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo

WORKDIR /root/robotx_gazebo/catkin_ws

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j4"; exit 0

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j4"; exit 0

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j4"; exit 0

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j4"; exit 0

WORKDIR /root/


