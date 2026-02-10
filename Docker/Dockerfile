FROM ros:noetic-ros-base-focal

# 1. Unified System Dependencies
# We install the system OpenCV, but NOT the system Eigen (since you need 3.1.0 from source)
RUN apt-get update && apt-get install -y \
    tmux sudo curl git cmake build-essential pkg-config v4l-utils \
    # Vision & Graphics
    libopencv-dev ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev \
    libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
    libgl1-mesa-dev libglew-dev libopengl-dev libgtk2.0-dev \
    # RealSense & Math Build Deps (Avoid libeigen3-dev here to prevent conflicts)
    libusb-1.0-0-dev libssl-dev libudev-dev \
    libcgal-dev libboost-all-dev libblas-dev liblapack-dev \
    # ROS
    ros-noetic-tf ros-noetic-image-transport ros-noetic-cv-bridge \
    ros-noetic-cv-camera \
    # Debugging
    libasan6 lsb-release wget software-properties-common gnupg nano

# more debug
RUN bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"

# 2. Build Eigen 3.1.0 from source (As requested)
RUN git clone https://gitlab.com/libeigen/eigen.git /tmp/eigen && \
    cd /tmp/eigen && git checkout 3.1.0 && \
    mkdir build && cd build && cmake .. && make install && \
    rm -rf /tmp/eigen

# 3. Build Pangolin v0.5
# Disabling Video (conflicts with FFmpeg 4) and Examples (linker errors with librt)
RUN git clone https://github.com/stevenlovegrove/Pangolin /tmp/Pangolin && \
    cd /tmp/Pangolin && git checkout tags/v0.5 && \
    mkdir build && cd build && \
    cmake .. \
        -DBUILD_PANGOLIN_VIDEO=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DCMAKE_CXX_FLAGS="-Wno-int-in-bool-context" && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Pangolin

# 4. Build librealsense v2.54.1 (As requested for specific flags)
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && git checkout v2.54.1 && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_NETWORK_DEVICE=OFF \
    && make -j$(nproc) && make install && \
    rm -rf /tmp/librealsense

# 5. Project Setup
COPY . /Line-Plane-CARV
WORKDIR /Line-Plane-CARV

# Ensure the system finds the source-built libs in /usr/local/lib
RUN ldconfig

# Rebuild project
RUN chmod +x build.sh && ./build.sh

# build main package
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; \
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/Line-Plane-CARV/Examples/ROS; \
    chmod +x build_ros.sh && ./build_ros.sh'

# Make Python work
RUN chmod +x /Line-Plane-CARV/ros_cam_publishers/scripts/mjpeg_camera_node.py && \
    chmod +x /Line-Plane-CARV/ros_cam_publishers/scripts/mp4_camera_node.py && \
    chmod +x /Line-Plane-CARV/ros_cam_publishers/scripts/tum_camera_node.py

# set up ROS for runtime

RUN printf 'source /Line-Plane-CARV/Examples/ROS/ORB_SLAM2/build/devel/setup.bash\n' >> /root/.bashrc
RUN printf 'export ROS_PACKAGE_PATH=/Line-Plane-CARV/Examples/ROS:/Line-Plane-CARV/ros_cam_publishers:${ROS_PACKAGE_PATH}\n' >> /root/.bashrc









RUN chmod +x /Line-Plane-CARV/core_camera.sh /Line-Plane-CARV/slam_carv.sh

# change default location to orbslamcarv
RUN echo "cd /Line-Plane-CARV" >> /root/.bashrc

# set up VNC server
ENV X11VNC_P="password"
RUN apt-get update -y
RUN apt-get install -y xvfb openbox xterm wget gnupg && \
    wget https://downloads.sourceforge.net/project/turbovnc/3.1/turbovnc_3.1_amd64.deb && \
    apt-get install -y ./turbovnc_3.1_amd64.deb && \
    rm turbovnc_3.1_amd64.deb
ADD ./Docker/entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh
ENTRYPOINT /opt/entrypoint.sh