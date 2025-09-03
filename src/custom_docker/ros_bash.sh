#!/bin/bash
set -e

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface name="eth0" priority="default" multicast="default" />
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>'

source ~/ros2_ws/install/setup.sh

# 전달된 명령 실행
exec "$@"

echo "ROS 2 Humble sourcing"