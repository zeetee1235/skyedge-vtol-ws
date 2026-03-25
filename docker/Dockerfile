# =============================================
# VTOL 프로젝트 통합 Docker 이미지
# 베이스: ROS 2 Humble (Ubuntu 22.04)
# =============================================
FROM ros:humble

# 비대화형 설치
ENV DEBIAN_FRONTEND=noninteractive

# 시스템 의존성 설치
# foxglove_bridge 는 별도 서비스에서 설치하므로 여기서는 제외한다.
RUN apt-get update -q && \
    apt-get install -y -q --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    wget \
    vim \
    ros-humble-vision-msgs \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

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
     git clone --depth 1 --branch release/1.17 https://github.com/PX4/px4_msgs.git /tmp/px4_msgs && \
     cp -r /tmp/px4_msgs /vtol_ws/src/px4_msgs && \
     rosdep install --from-paths src --ignore-src -r -y && \
     colcon build --symlink-install"

CMD ["/bin/bash"]
