#!/bin/bash

# probably adjust dev video0
docker run \
  -p 5901:5901 \
  -p 8080:8080 \
  --tmpfs /tmp:rw,size=100m \
  --tmpfs /root/.vnc \
  --rm \
  --privileged \
  -e DISPLAY=:$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --device=/dev/video0:/dev/video0 \
  -v /temp/.X11-unix \
  -it imjunaida/line_plane_carv-docker:v1.0
