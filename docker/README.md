# Gym-Gazebo Docker
There are two catagories of dockerfiles in this repo: base images and example images.  The base images are simply versions of the gym-gazebo images with only the nessesary support libraries such as ROS and Gym.

The application images are built on the base images described above and have examples loaded into them.

This fodler is setup such that each image has its own folder is an image and each subfolder is a tagged version.  This enbles automated build in https://hub.docker.com. For example:

## If you are working on a gpu machine
```
xhost +"local:docker@"
docker pull d3637042/ros-kinetic-gazebo-gym-nvidia9.0:latest
docker run \
    --runtime=nvidia \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --rm \
    -it \
    -p 11345:11345 \
    -p 11311:11311 \
    --name=ros-kinetic-gazebo-gym-nvidia9.0 \
    d3637042/ros-kinetic-gazebo-gym-nvidia9.0:latest \
    /bin/bash
```
This will open a bash shell in the container:
```
python /root/gym-gazebo/examples/wamv/robotx_lidar_qlearn.py
```
## If you are working on a non-gpu machine
d3637042/ros-kinetic-gazebo-gym
```
xhost +"local:docker@"
docker pull d3637042/ros-kinetic-gazebo-gym:latest
docker run \
    --runtime=nvidia \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --rm \
    -it \
    -p 11345:11345 \
    -p 11311:11311 \
    --name=ros-kinetic-gazebo-gym \
    d3637042/ros-kinetic-gazebo-gym:latest \
    /bin/bash
```
This will open a bash shell in the container:
```
python /root/gym-gazebo/examples/wamv/robotx_lidar_qlearn.py
```

## If you want to use local gazebo: 
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.IPAddress }}' ros-kinetic-gazebo-gym-nvidia9.0 ) && export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345 && sudo gzclient --verbose
```


