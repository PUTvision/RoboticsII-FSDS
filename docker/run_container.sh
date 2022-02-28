#!/bin/bash

if [[ $# -eq 0 ]] ; then
    echo "Container name (student ID) not passed!"
    exit 1
fi

docker run \
    -it --gpus all --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_DISABLE_REQUIRE=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host \
    --name=$1 \
    fsds
