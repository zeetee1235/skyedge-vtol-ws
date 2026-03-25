# =============================================
# Micro XRCE-DDS Agent 전용 이미지
# PX4 SITL ↔ ROS2 브리지 용도
#
# ROS2 Humble의 fastdds(2.6.x)를 재사용해
# DDS discovery 호환성 보장
# =============================================
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -y -q --no-install-recommends \
    git \
    cmake \
    g++ \
    build-essential \
    libasio-dev \
    libtinyxml2-dev \
    ros-humble-fastrtps-cmake-module \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v2.4.3 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent && \
    . /opt/ros/humble/setup.sh && \
    cmake -S /tmp/xrce_agent -B /tmp/xrce_agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/xrce_agent/build --parallel "$(nproc)" && \
    cmake --install /tmp/xrce_agent/build && \
    ldconfig && \
    rm -rf /tmp/xrce_agent

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && MicroXRCEAgent udp4 -p 8888"]
