# docker build -t austinderic/gym-gazebo:kinetic .
FROM osrf/ros:kinetic-desktop-full

# set environment variables
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Dependencies for building packages
RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python-pip \
    nano \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Unofficial ROS installation steps
RUN pip install --upgrade pip

RUN pip install -U catkin_tools

#--------------------
# Install OpenAI gym
#-------------------

RUN pip install gym

#--------------------
# Install gym-gazebo
#--------------------

RUN git clone -b master https://github.com/erlerobot/gym-gazebo.git /root/gym-gazebo
WORKDIR /root/gym-gazebo
RUN pip install -e .

