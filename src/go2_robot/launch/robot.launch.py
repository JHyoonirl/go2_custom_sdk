import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')
    with_slam = LaunchConfiguration('slam', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')

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
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("go2_robot"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

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
    
    go2_sport_ctrl_node = Node(
        package='go2_robot',
        executable='go2_sport_ctrl_node',
        # parameters=[{'robot_ip': robot_ip, "conn_type": conn_type}],
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
            'min_height': -0.15,         # 바닥면 노이즈 제거를 위해 추가
            'max_height': 0.15,          # 천장이나 너무 높은 장애물 제거
            'range_min': 0.1,          # 로봇에 너무 가까운 포인트(노이즈) 제거
            'range_max': 2.0,          # 최대 감지 거리 설정
            'use_inf': True,            # 측정 범위를 벗어난 거리를 무한대(inf)로 처리
        }],
        output='screen',
    )
    
    start_async_slam_toolbox_node = Node(
        condition=IfCondition(with_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[('/odom', '/utlidar/robot_odom')],  # PointCloud to LaserScan 노드의 출력 토픽과 일치시킴
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
        declare_slam_params_file_cmd,
        robot_state_publisher_node,
        go2_driver_node,
        go2_sport_ctrl_node,
        pointcloud_to_laserscan_node,
        start_async_slam_toolbox_node,
        rviz2_node,
    ])