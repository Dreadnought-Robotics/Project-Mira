docker run -it \
  -v ~/Project-Mira:/root/Mira/ \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
#   --cpus=4  \
#   --memory=6g \
  --device /dev/video0:/dev/video0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  mira-docker:latest bash

