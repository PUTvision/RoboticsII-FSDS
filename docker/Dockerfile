# use image with cuda drivers and vulkan
FROM nvidia/vulkan:1.2.133-450

ARG DEBIAN_FRONTEND=noninteractive

# set time zone
ENV TZ=Europe/Warsaw
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rm /etc/apt/sources.list.d/cuda.list
RUN apt-key del 7fa2af80
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb && \
	dpkg -i cuda-keyring_1.0-1_all.deb
RUN apt-get update
RUN apt install -y tzdata

# update libraries
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
	python3 \
	python3-pip \
	sudo \
	libglu1-mesa-dev \
	xdg-user-dirs \
	pulseaudio \
	sudo \
	x11-xserver-utils \
	git \
    git-lfs \
    wget \
	rsync \
	unzip \
	g++ \
	vim \
    lsb-release \
    vulkan-utils \
    python3-tk \
    dirmngr \
    gnupg2 \
    keyboard-configuration

# Enable Vulkan support
ARG VULKAN_VERSION="1.2.148"
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility,display
RUN export $(cat /etc/os-release | grep --color=never 'UBUNTU_CODENAME') && \
    wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - && \
    wget -qO /etc/apt/sources.list.d/lunarg-vulkan-${VULKAN_VERSION}-${UBUNTU_CODENAME}.list \
    https://packages.lunarg.com/vulkan/${VULKAN_VERSION}/lunarg-vulkan-${VULKAN_VERSION}-${UBUNTU_CODENAME}.list && \
    apt-get update && apt-get install -y --no-install-recommends vulkan-sdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
    mkdir -p /etc/vulkan/icd.d/ && \
    echo \
    "{\
        \"file_format_version\" : \"1.0.0\",\
        \"ICD\": {\
            \"library_path\": \"libGLX_nvidia.so.0\",\
            \"api_version\" : \"${VULKAN_API_VERSION}\"\
        }\
    }" > /etc/vulkan/icd.d/nvidia_icd.json

# install python libs
RUN pip3 install setuptools wheel
RUN pip3 install --quiet numpy==1.18.5 rospkg pyyaml pyquaternion scipy msgpack-rpc-python
RUN pip3 install --quiet scikit-build
RUN pip3 install --quiet airsim

RUN pip3 install --quiet onnxruntime==1.10.0
RUN pip3 install --quiet torch==1.10.2+cpu torchvision==0.11.3+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html

# install ros noetic and requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    sudo apt update && \
    sudo apt install -y ros-noetic-desktop && \
	sudo apt install -y ros-noetic-tf2-geometry-msgs \
		python3-catkin-tools \
		ros-noetic-rqt-multiplot \
		ros-noetic-joy \
		ros-noetic-cv-bridge \
		ros-noetic-image-transport \
		ros-noetic-ros-numpy \
		ros-noetic-robot-localization \
		ros-noetic-vision-msgs \
		ros-noetic-geodesy \
		ros-noetic-pcl-ros \
		ros-noetic-nmea-msgs \
		ros-noetic-libg2o \
		libyaml-cpp-dev \
		libcurl4-openssl-dev \
		python3-pcl

RUN adduser --force-badname --disabled-password --gecos '' --shell /bin/bash fsdssim_user && \ 
	echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \ 
	adduser fsdssim_user sudo && \ 
	adduser fsdssim_user audio && \ 
	adduser fsdssim_user video

USER fsdssim_user
WORKDIR /home/fsdssim_user

# link python3 for ROS noetic
RUN echo "alias python='python3'" >> ~/.bashrc

# add ros melodic to startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# get FSDS
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git -b v2.1.0 --recurse-submodules --quiet

ADD ./files/settings.json Formula-Student-Driverless-Simulator/settings.json
ADD ./files/default.rviz Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/rviz/default.rviz
ADD ./files/cameralauncher.py Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/scripts/cameralauncher.py
ADD ./files/fsds_ros_bridge_camera.cpp Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/src/fsds_ros_bridge_camera.cpp

# build FSDS
RUN cd Formula-Student-Driverless-Simulator/AirSim && \
    ./setup.sh && \
    ./build.sh

# clone race evaluator
RUN cd Formula-Student-Driverless-Simulator/ros/src && \
	git clone https://github.com/bartoszptak/ros-driving-track-evaluator.git -b sim/fsds --quiet && \
	git clone https://github.com/PUTvision/RoboticsII-FSDS.git -b ros --quiet && \
	git clone https://github.com/koide3/ndt_omp.git --quiet && \
	git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive --quiet && \
	git clone https://github.com/koide3/hdl_graph_slam --quiet

# build ROS bridge
RUN cd Formula-Student-Driverless-Simulator/ros && \
	catkin config --extend /opt/ros/noetic  && \	
    catkin init && \
	LINE=`sed -n '/.publish(go_signal_msg)/=' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp` && \
	sed -i "$LINE"' s/^/\/\//' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp && \
    catkin build

# add FSDS env to startup
RUN echo "source /home/fsdssim_user/Formula-Student-Driverless-Simulator/ros/devel/setup.bash" >> ~/.bashrc

# install python requrements
RUN cd Formula-Student-Driverless-Simulator/python && pip3 install -r requirements.txt

RUN wget "https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.1.0/fsds-v2.1.0-linux.zip" && \
	unzip fsds-v2.1.0-linux.zip && rm fsds-v2.1.0-linux.zip
