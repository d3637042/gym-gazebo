# Gym-Gazebo Docker
There are two catagories of dockerfiles in this repo: base images and example images.  The base images are simply versions of the gym-gazebo images with only the nessesary support libraries such as ROS and Gym.

The application images are built on the base images described above and have examples loaded into them.

This fodler is setup such that each image has its own folder is an image and each subfolder is a tagged version.  This enbles automated build in https://hub.docker.com. For example:



## Requirements
[nvidia-docker 2](https://github.com/NVIDIA/nvidia-docker) allows running cuda-based nueral nets and render displays from the container.


## Recommended (Easy) Method
```
docker run \
    --runtime=nvidia \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --rm \
    -it \
    -p 11345:11345 \
    -p 11311:11311 \
    --name=gym-gazebo-turtlebot \
    austinderic/gym-gazebo-turtlebot:kinetic-nvidia9.2 \
    /bin/bash
```
This will open a bash shell in the container:
```
python ./gym-gazebo/examples/turtlebot/circuit2_turtlebot_lidar_qlearn.py
```
then we can connect to our new container 
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.IPAddress }}' gym-gazebo-turtlebot)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
sudo gzclient --verbose
```

Run tests in the base image. Simply enter the following command, all you need is nvidia-docker2 setup:
```
docker run --runtime=nvidia --rm austinderic/gym-gazebo:kinetic-nvidia9.2 /bin/bash -c "nvidia-smi; echo $(rosversion -d); pip show gym; pip show gym-gazebo"
``` 
Even easier with docker-compose:
```
docker-compose up
```

## Manual Method
```
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src
wstool initW
wstool merge https://raw.githubusercontent.com/turtlebot/turtlebot/kinetic/turtlebot.rosinstall && wstool merge ../../gym-gazebo/gym_gazebo/envs/installation/gym-gazebo.rosinstall
```

