import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')

    robot_ip = os.getenv('ROBOT_IP', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    rviz_config = "cyclonedds_config.rviz"

    # --- URDF 및 robot_state_publisher 설정 ---
    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(
        get_package_share_directory('go2_robot'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', # 이름에서 _robot 접미사 제거 (일반적)
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}]
        # Foxy 버전에서는 arguments=[urdf]가 필요하지 않을 수 있습니다. 
        # robot_description 파라미터로 충분합니다.
    )

    # --- 드라이버 노드 ---
    go2_driver_node = Node(
        package='go2_robot',
        executable='go2_driver_node',
        parameters=[{'robot_ip': robot_ip, "conn_type": conn_type}],
    )

    # --- PointCloud to LaserScan 노드 (수정된 부분) ---
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', # 이름 단순화
        remappings=[
            # 'could_deskewed' -> 'cloud_deskewed' 오타 수정
            ('cloud_in', '/utlidar/cloud_deskewed'), 
            ('scan', '/scan'), # 출력 토픽 이름을 명확히 '/scan'으로 지정
        ],
        parameters=[{
            'target_frame': 'base_link', # 스캔 데이터가 생성될 좌표계
            'min_height': 0.5,         # 바닥면 노이즈 제거를 위해 추가
            'max_height': 0.7,          # 천장이나 너무 높은 장애물 제거
            'range_min': 0.3,           # 로봇에 너무 가까운 포인트(노이즈) 제거
            'use_inf': True,            # 측정 범위를 벗어난 거리를 무한대(inf)로 처리
        }],
        output='screen',
    )
    
    # --- RViz2 노드 ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(with_rviz2),
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('go2_robot'), 'config', rviz_config)]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        go2_driver_node,
        pointcloud_to_laserscan_node,
        rviz2_node,
    ])