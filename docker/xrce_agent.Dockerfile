# =============================================
# Micro XRCE-DDS Agent 전용 이미지
# PX4 SITL ↔ ROS2 브리지 용도
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
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch v2.4.2 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent && \
    cmake -S /tmp/xrce_agent -B /tmp/xrce_agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/xrce_agent/build --parallel "$(nproc)" && \
    cmake --install /tmp/xrce_agent/build && \
    rm -rf /tmp/xrce_agent

CMD ["MicroXRCEAgent", "udp4", "-p", "8888"]
