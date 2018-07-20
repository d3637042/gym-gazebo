FROM austinderic/ros-desktop-full:kinetic-nvidia9.2
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install OpenAI gym
#-------------------

RUN pip install gym

#--------------------
# Install gym-gazebo
#--------------------

RUN git clone https://github.com/erlerobot/gym-gazebo.git /root/gym-gazebo
WORKDIR /root/gym-gazebo
RUN pip install -e .




