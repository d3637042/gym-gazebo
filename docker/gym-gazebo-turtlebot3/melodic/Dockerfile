FROM austinderic/gym-gazebo:melodic
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install Gym-Gazebo Turtlebot Example
#------------------

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN wstool init
RUN wstool merge https://gist.githubusercontent.com/AustinDeric/8bb2648e9281d62c7055911e7cedcdae/raw/da49a1a014cd106dbae1e792224abc5efe4d2c31/gym-gazebo-turtlebot3-melodic.rosinstall
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

