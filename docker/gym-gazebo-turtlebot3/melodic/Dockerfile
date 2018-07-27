FROM austinderic/gym-gazebo:melodic
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install Gym-Gazebo Turtlebot3 dependencies
#------------------

RUN apt-get update && apt-get install -y \
    ros-melodic-geometry2 \
    ros-melodic-xacro \
    ros-melodic-roslint \
    ros-melodic-ros-control \
    ros-melodic-image-common \
    ros-melodic-control-toolbox \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-roslint \
    ros-melodic-joystick-drivers \
    ros-melodic-ecl \
    ros-melodic-pcl-ros \
    ros-melodic-yocs-msgs \
    libbullet-dev \
    libusb-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    libftdi-dev \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip install scikit-image

#--------------------
# Install Gym-Gazebo Turtlebot Example
#------------------

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN wstool init
RUN wstool merge https://gist.githubusercontent.com/AustinDeric/c14633f1d5bcabc14307dcca2425d133/raw/266668674782d5e1096ecd13f3ea6dcf5af5248a/gym-gazebo-turtlebot-melodic.rosinstall
RUN wstool update
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin build"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

#--------------------
# Setup environment variables
#------------------
RUN echo "export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world" >> ~/.bashrc
RUN export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world
RUN echo "export GAZEBO_MODEL_PATH=/root/gym-gazebo/gym_gazebo/envs/assets/models" >> ~/.bashrc

WORKDIR /root/

