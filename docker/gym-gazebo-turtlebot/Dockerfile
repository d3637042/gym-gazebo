FROM austinderic/gym-gazebo:kinetic-nvidia9.2
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install Gym-Gazebo Turtlebot dependencies
#------------------

RUN apt-get update && apt-get install -y \
    ros-kinetic-turtlebot-create \
    ros-kinetic-yujin-ocs \
    ros-kinetic-yocs-msgs \
    ros-kinetic-xacro \
    ros-kinetic-roslint \
    ros-kinetic-ros-control \
    ros-kinetic-image-common \
    ros-kinetic-control-toolbox \
    ros-kinetic-gazebo-ros-pkgs \
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
    libbullet-dev \
    libusb-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    python-skimage \
    && rm -rf /var/lib/apt/lists/*

#--------------------
# Install Gym-Gazebo Turtlebot Example
#------------------

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN wstool init
RUN wstool merge https://gist.githubusercontent.com/AustinDeric/3362c86437d88347807beaba3501602e/raw/ecadfddc98a1865dfe60b29d17ff03a513b39279/gym-gazebo.rosinstall
RUN wstool update
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin build"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

#--------------------
# Setup environment variables
#------------------
RUN echo "export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world" >> ~/.bashrc
RUN export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world

WORKDIR /root/
