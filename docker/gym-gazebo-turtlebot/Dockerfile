FROM austinderic/gym-gazebo-turtlebot:kinetic-nvidia9.2
LABEL maintainer "Austin.Deric@gmail.org"

#--------------------
# Install Turtlebot dependencies
#-------------------

#--------------------
# Install Gym-Gazebo System Dependencies 
#-------------------

RUN apt-get update && apt-get install -y \
    libusb-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    && rm -rf /var/lib/apt/lists/*
