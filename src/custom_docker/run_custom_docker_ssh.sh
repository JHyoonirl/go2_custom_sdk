#!/bin/bash

# Docker 컨테이너를 실행하고 즉시 내부 쉘로 접속하는 스크립트

DOCKER_IMAGE="my-ros-humble"
CONTAINER_NAME="my-ros-interactive" # 이름은 겹치지 않게 변경

SHARED_FOLDER="$HOME/go2_custom_sdk"
CONTAINER_FOLDER="/root/ros2_ws"

# X11 Forwarding 설정 확인
if [ -z "$DISPLAY" ]; then
    echo "오류: DISPLAY 환경 변수가 설정되지 않았습니다. ssh -Y 옵션으로 접속했는지 확인하세요."
    exit 1
fi

echo "ROS 2 컨테이너를 시작하고 즉시 접속합니다..."

# 혹시 같은 이름의 컨테이너가 있다면 사전 정리
docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1

# -it: 컨테이너와 상호작용하는 터미널 모드
# --rm: 컨테이너 종료 시 자동 삭제
# 마지막의 'bash'는 컨테이너 시작 시 실행할 명령어를 의미 (기본값이 bash일 수 있지만 명시하는 것이 안전)
docker run -it --rm \
    --name my-ros-interactive \
    --network host \
    --gpus all \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v "$HOME/go2_custom_sdk:/root/ros2_ws" \
    --workdir /root/ros2_ws \
    my-ros-humble \
    bash
echo "컨테이너 세션이 종료되었습니다."
