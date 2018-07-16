FROM austinderic/ros-desktop-full:kinetic-nvidia9.2
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Setup nvidia-docker2
#--------------------
# Reference: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#--------------------
# Install OpenAI gym
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

RUN git clone https://github.com/erlerobot/gym-gazebo.git /root/gym-gazebo
WORKDIR /root/gym-gazebo
RUN pip install -e .




