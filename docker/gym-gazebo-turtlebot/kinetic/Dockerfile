# docker build -t austinderic/gym-gazebo-turtlebot:kinetic
FROM austinderic/gym-gazebo:kinetic
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install gzweb
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
# Install Gym-Gazebo Turtlebot dependencies
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
    mesa-utils \
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
RUN echo "export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world" >> ~/.bashrc
RUN export GYM_GAZEBO_WORLD_CIRCUIT2=/root/gym-gazebo/gym_gazebo/envs/assets/worlds/circuit2.world
RUN echo "export GAZEBO_MODEL_PATH=/root/gym-gazebo/gym_gazebo/envs/assets/models" >> ~/.bashrc

COPY ./kobuki_description/ /root/gzweb/http/client/assets/kobuki_description/
COPY ./turtlebot_description/ /root/gzweb/http/client/assets/turtlebot_description/

WORKDIR /root/

