docker run -it --rm \
  --runtime=nvidia \
  --env DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env XAUTHORITY=$XAUTH \
  --volume "$XAUTH:$XAUTH" \
  --volume "/tmp/.X11-unix:/tmp/.X11-unix" \
  austinderic/gym-gazebo:melodic \
  /bin/bash
