#!/bin/bash

# ROS 2 Humble Docker 컨테이너를 GUI 옵션과 함께 실행하는 스크립트

# Docker 이미지 이름 설정
DOCKER_IMAGE="my-ros-humble"

# --- 폴더 공유 설정 ---
# 로컬의 ~/ros_ws 폴더를 컨테이너의 /root/ros_ws 폴더에 연결합니다.
# 이 경로를 원하는 폴더로 변경하여 사용하세요.
SHARED_FOLDER="$HOME/ros2_ws"
CONTAINER_FOLDER="/root/ros2_ws"
# --------------------

# X11 Forwarding을 위한 DISPLAY 환경 변수와 .Xauthority 파일 경로 확인
if [ -z "$DISPLAY" ]; then
    echo "오류: DISPLAY 환경 변수가 설정되지 않았습니다."
    echo "X11 서버가 실행 중인지 확인해주세요."
    exit 1
fi

XAUTH_FILE="$HOME/.Xauthority"
if [ ! -f "$XAUTH_FILE" ]; then
    echo "오류: $XAUTH_FILE 파일을 찾을 수 없습니다."
    exit 1
fi

echo "ROS 2 Humble 컨테이너를 시작합니다..."
echo "이미지: $DOCKER_IMAGE"
echo "공유 폴더: $SHARED_FOLDER -> $CONTAINER_FOLDER"
echo "-------------------------------------"


# 로컬에 공유할 폴더가 없다면 생성
mkdir -p "$SHARED_FOLDER"

# Docker 컨테이너 실행
# --network host: 컨테이너가 호스트의 네트워크를 직접 사용
# -e DISPLAY=$DISPLAY: 호스트의 DISPLAY 환경 변수를 컨테이너로 전달하여 GUI를 표시할 위치를 지정
# -v $HOME/.Xauthority:/root/.Xauthority: X server 인증을 위한 파일을 공유
# --rm: 컨테이너 종료 시 자동으로 삭제
# -it: 대화형 TTY 모드로 실행
docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v "$XAUTH_FILE:/root/.Xauthority" \
    -v "$SHARED_FOLDER:$CONTAINER_FOLDER" \
    --workdir "$CONTAINER_FOLDER" \
    "$DOCKER_IMAGE"

echo "-------------------------------------"
echo "컨테이너가 종료되었습니다."