FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── Base tools ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    apt-transport-https \
    ca-certificates \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libtbb-dev \
    libpcl-dev \
    libopencv-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    python3-dev \
    python3-numpy \
    python3-matplotlib \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 1 Noetic ─────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-tf \
    ros-noetic-tf2-msgs \
    ros-noetic-tf-conversions \
    ros-noetic-eigen-conversions \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-message-filters \
    ros-noetic-rosbag \
    ros-noetic-rosbag-storage \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# ── rosbags (used to convert ROS 2 bags to ROS 1, if needed) ─────────────────
RUN pip3 install --no-cache-dir "rosbags==0.9.22"

# ── GTSAM 4.0.3 ──────────────────────────────────────────────────────────────
#    DA-LIO's CMakeLists hard-codes the prefix /home/third_library/gtsam-4.0.3/install,
#    so install there. Build with system Eigen and WITHOUT -march=native.
RUN mkdir -p /home/third_library && cd /home/third_library && \
    wget -q -O gtsam-4.0.3.tar.gz https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.tar.gz && \
    tar -zxf gtsam-4.0.3.tar.gz && \
    cd gtsam-4.0.3 && mkdir build install && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF \
      -DCMAKE_INSTALL_PREFIX=/home/third_library/gtsam-4.0.3/install && \
    make -j$(nproc) && make install && \
    cp /home/third_library/gtsam-4.0.3/install/lib/libmetis-gtsam.so /usr/lib/ 2>/dev/null || true && \
    echo "/home/third_library/gtsam-4.0.3/install/lib" > /etc/ld.so.conf.d/gtsam.conf && \
    ldconfig && \
    cd / && rm -rf /home/third_library/gtsam-4.0.3.tar.gz /home/third_library/gtsam-4.0.3/build

# ── Ceres 2.1.0 ──────────────────────────────────────────────────────────────
#    DA-LIO's CMakeLists hard-codes /home/third_library/ceres-solver-2.1.0/install.
RUN cd /home/third_library && \
    wget -q -O ceres-2.1.0.tar.gz https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz && \
    tar -zxf ceres-2.1.0.tar.gz && \
    cd ceres-solver-2.1.0 && mkdir build install && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DCMAKE_INSTALL_PREFIX=/home/third_library/ceres-solver-2.1.0/install && \
    make -j$(nproc) && make install && \
    echo "/home/third_library/ceres-solver-2.1.0/install/lib" > /etc/ld.so.conf.d/ceres.conf && \
    ldconfig && \
    cd / && rm -rf /home/third_library/ceres-2.1.0.tar.gz /home/third_library/ceres-solver-2.1.0/build

# ── Livox-SDK (required by livox_ros_driver) ─────────────────────────────────
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK/build && \
    cmake .. && \
    make -j$(nproc) && make install && \
    ldconfig && \
    cd / && rm -rf /tmp/Livox-SDK

# ── Build catkin workspace (DA-LIO + livox_ros_driver + converter) ───────────
WORKDIR /ros_ws

COPY ./src/DALI_SLAM             ./src/DALI_SLAM
COPY ./src/livox_ros_driver      ./src/livox_ros_driver
COPY ./src/dalislam-to-hdmapping ./src/dalislam-to-hdmapping

# Benchmark launch (selectable config) added into the DA_LIO package.
COPY ./overlay/launch/ ./src/DALI_SLAM/DA_LIO/launch/

# Build the livox_ros_driver message (CustomMsg) first, then DA-LIO and the
# converter. Only da_lio is built from the DALI_SLAM repo (the MC_PGO back-end
# package and its extra dependencies are not needed for this benchmark), hence
# the --only-pkg-with-deps whitelist for each stage. The final guard fails the
# image build if either executable is missing.
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make --only-pkg-with-deps livox_ros_driver      -DCMAKE_BUILD_TYPE=Release -j$(nproc) && \
    catkin_make --only-pkg-with-deps da_lio                -DCMAKE_BUILD_TYPE=Release -j$(nproc) && \
    catkin_make --only-pkg-with-deps dalislam_to_hdmapping -DCMAKE_BUILD_TYPE=Release -j$(nproc) && \
    test -f /ros_ws/devel/lib/da_lio/da_lio && \
    test -f /ros_ws/devel/lib/dalislam_to_hdmapping/listener && \
    echo "[build] da_lio node and converter present"

# ── Non-root user ─────────────────────────────────────────────────────────────
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros && \
    chown -R $UID:$GID /ros_ws

RUN echo "source /opt/ros/noetic/setup.bash"   >> /root/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"     >> /root/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash"   >> /home/ros/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"     >> /home/ros/.bashrc

CMD ["bash"]
