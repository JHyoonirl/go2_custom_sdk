# NVIDIA L4T Base 이미지를 사용 (JetPack 6.x, Ubuntu 22.04 기반)
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r36.2.0
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=humble

# 로케일 및 시간대 설정 (ROS2 설치에 필요)
ENV DEBIAN_FRONTEND=noninteractive
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 필수 패키지 설치, ROS 저장소 설정, ROS 패키지 설치를 하나의 RUN 명령어로 통합
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    locales \
    tzdata \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    git \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    software-properties-common \
    ffmpeg \
    ca-certificates \
    && \
    # 로케일 설정
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    # 캐시 정리
    rm -rf /var/lib/apt/lists/*


    # --- 2. ROS 저장소 설정 ---
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# --- 3. ROS 및 기타 Python 패키지 설치 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    && \
    pip install --no-cache-dir 'numpy<2.0.0' && \
    rm -rf /var/lib/apt/lists/*
    
RUN pip install 'numpy<2.0.0'

# --- 4. go2_sdk 소스코드 다운 및 의존성 설치 (사용자가 질문한 부분) ---
# 작업 디렉토리를 미리 만들고 이동합니다.
WORKDIR /ros2_ws

# 소스 코드를 src 디렉토리 아래에 클론합니다.
RUN git clone --depth 1 https://github.com/JHyoonirl/go2_custom_sdk.git src/go2_custom_sdk

# Python 의존성과 ROS 의존성을 설치합니다.
# '.' 은 'source'와 같은 역할을 합니다.
RUN ["/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    pip install --no-cache-dir -r src/go2_custom_sdk/requirements.txt && \
    apt-get update && \
    rosdep init && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*" ]

# ROS 2 환경 설정
COPY ./ros_bash.sh /
RUN chmod +x /ros_bash.sh
ENTRYPOINT ["/ros_bash.sh"]
CMD ["bash"]