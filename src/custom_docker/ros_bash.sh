#!/bin/bash
set -e

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 전달된 명령 실행
exec "$@"