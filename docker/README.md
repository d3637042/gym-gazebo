# Gym-Gazebo Docker
There are two catagories of dockerfiles in this repo: base images and example images.  The base images are simply versions of the gym-gazebo images with only the nessesary support libraries such as ROS and Gym.

The application images are built on the base images described above and have examples loaded into them.

This fodler is setup such that each image has its own folder is an image and each subfolder is a tagged version.  This enbles automated build in https://hub.docker.com. For example:



## Requirements
[nvidia-docker 2](https://github.com/NVIDIA/nvidia-docker) allows running cuda-based nueral nets and render displays from the container.


## Recommended (Easy) Method
```
xhost +"local:docker@"
docker run \
    --runtime=nvidia \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --rm \
    -it \
    -p 11345:11345 \
    -p 11311:11311 \
    --name=ros-kinetic-gazebo-gym-nvidia9.0 \
    os-kinetic-gazebo-gym-nvidia9.0:latest \
    /bin/bash
```
This will open a bash shell in the container:
```
python /root/gym-gazebo/examples/wamv/robotx_lidar_qlearn.py
```

then we can connect to our new container 
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.IPAddress }}' ros-kinetic-gazebo-gym-nvidia9.0 ) && export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345 && sudo gzclient --verbose
```


