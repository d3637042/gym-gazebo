# Gym-Gazebo Docker
There are two catagories of dockerfiles in this repo: base images and example images.  The base images are simply versions of the gym-gazebo images with only the nessesary support libraries such as ROS and Gym.

The application images are built on the base images described above and have examples loaded into them.

This fodler is setup such that each image has its own folder is an image and each subfolder is a tagged version.  This enbles automated build in https://hub.docker.com. For example:



## Requirements
nvidia-docker 2
https://github.com/NVIDIA/nvidia-docker

Allows running cuda-based nueral nets and render displays from the container.

## Recommended (Easy) Method
Run tests in the base image. Simply enter the following command, all you need is nvidia-docker2 setup:
```
docker run --runtime=nvidia --rm austinderic/gym-gazebo:kinetic-nvidia9.2 /bin/bash -c "nvidia-smi; echo $(rosversion -d); pip show gym; pip show gym-gazebo"
```

## Manual Method
```
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src
wstool initW
wstool merge https://raw.githubusercontent.com/turtlebot/turtlebot/kinetic/turtlebot.rosinstall && wstool merge ../../gym-gazebo/gym_gazebo/envs/installation/gym-gazebo.rosinstall
```
