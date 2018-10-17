docker run \
  --runtime=nvidia \
    -v="/tmp/.gazebo/:/root/.gazebo/" \
    --rm \
    -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -p 11345:11345 \
    -p 11311:11311 \
    --name=ros-kinetic-gazebo-gym-nvidia9.0 \
    d3637042/ros-kinetic-gazebo-gym-nvidia9.0:latest \
    /bin/bash
