# =============================================
# VTOL 프로젝트 통합 Docker 이미지
# 베이스: ROS 2 Humble (Ubuntu 22.04)
# =============================================
FROM ros:humble

# 비대화형 설치
ENV DEBIAN_FRONTEND=noninteractive

# 시스템 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    wget \
    vim \
    # Micro XRCE-DDS Agent 빌드 도구
    cmake \
    g++ \
    build-essential \
    # ROS2 패키지
    ros-humble-foxglove-bridge \
    ros-humble-vision-msgs \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    # 시리얼 통신 (아두이노)
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent (PX4 ↔ ROS2 UDP 브리지)
RUN git clone --depth 1 --branch v2.4.2 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent \
    && cmake -S /tmp/xrce_agent -B /tmp/xrce_agent/build \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/xrce_agent/build --parallel "$(nproc)" \
    && cmake --install /tmp/xrce_agent/build \
    && rm -rf /tmp/xrce_agent

# Python 패키지 설치
RUN pip3 install \
    opencv-python \
    ultralytics \
    numpy \
    pyserial

# rosdep 초기화
RUN rosdep init || true && rosdep update

# 워크스페이스 설정
WORKDIR /vtol_ws

# ROS2 환경 자동 소싱
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /vtol_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

# 빌드
COPY src/ /vtol_ws/src/
RUN /bin/bash -c \
    "set -e && \
     source /opt/ros/humble/setup.bash && \
     git clone --depth 1 https://github.com/PX4/px4_msgs.git /tmp/px4_msgs && \
     cp -r /tmp/px4_msgs /vtol_ws/src/px4_msgs && \
     rosdep install --from-paths src --ignore-src -r -y && \
     colcon build --symlink-install"

CMD ["/bin/bash"]
