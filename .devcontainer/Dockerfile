FROM athackst/ros2:foxy-gazebo

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    #python3-colcon-common-extensions\
    ros-foxy-diagnostic-updater \
    ros-foxy-rqt \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    iputils-ping \
    jq \
    openssh-server \
    sshpass\
    git-lfs \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Setup communication with MCU
RUN cd /home/ros && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    git checkout foxy && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/

ENV DEBIAN_FRONTEND=dialog

RUN service ssh start

EXPOSE 22

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc