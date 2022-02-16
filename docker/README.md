# Formula Student Driverless Simulator docker image

Directory contains docker image with Formula Student Driverless Simulator which is based on:
- [Formula-Student-Driverless-Simulator](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/latest/getting-started/)
- [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [NVIDIA docker image with Vulkan and CUDAGL](https://hub.docker.com/r/nvidia/vulkan/tags)

<p align="center">
  <img width="960" height="540" src="https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/docs/images/banner.png?raw=true">
</p>

## Requirements

### Hardware

*  recommended system requirements:
    - 8 core 2.3Ghz CPU
    - 12 GB memory
    - 30GB free SSD storage
    - NVidia card with Vulkan support and 3 GB of memory

* testing machine:
    - OS: Ubuntu 20.04.1 LTS 64-bit
    - Intel® Core™ i5-8400 CPU @ 2.80GHz × 6
    - RAM 15.6 GiB
    - GeForce GTX 1060 6GB

* docker image size is about 9 GB


### Software

- docker >= 19.03
- NVIDIA GPU - [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) section installation on Ubuntu


## Instalation

0. Clone repository

```bash
git clone https://github.com/PUTvision/RoboticsII-FSDS.git
```

1. Build docker image from [docker](../../docker) directory

```bash
docker build -t fsds --network=host -f Dockerfile .
```

2. Add docker access to nvidia (it's require sudo privileges to execute)

```bash
chmod +x xauth.sh
./xauth.sh
```

3. Run docker image

```bash
docker run \
    -it --gpus all --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host \
    --name=robotisc2-fsds \
    fsds
```

It is also possible using [run_container.sh](./run_container.sh) script.

4. Get into the container from other terminal

```bash
docker exec -it <CONTAINER ID> bash
```

It is also possible using [exec_to_container.sh](./exec_to_container.sh) script.
