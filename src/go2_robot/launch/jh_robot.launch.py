import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')
    with_slam = LaunchConfiguration('slam', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    serial_no = LaunchConfiguration('serial_no', default="'233722072808'")
    config_file_path = LaunchConfiguration('config_file', default=os.path.join(
        get_package_share_directory('go2_robot'), # YAML 파일이 있는 패키지 이름
        'config',                                  # config 폴더
        'my_realsense_config.yaml'                 # YAML 파일 이름
    ))
    yolo_model_path = LaunchConfiguration('yolo_model_path', default=os.path.join(
        get_package_share_directory('go2_robot'), # YAML 파일이 있는 패키지 이름
        'yolo_models',                             # yolo_models 폴더
        'yolov8s.engine'                           # YOLO 엔진 파일 이름
    ))
    # libgomp_path = "/usr/lib/aarch64-linux-gnu/libgomp.so.1" # <-- 사용자 시스템에서 찾은 경로로 바꾸세요!
    ld_preload_libs = [
    # 1. 오류의 원인인 libGLdispatch.so.0 (가장 먼저 로드되도록 설정)
    '/lib/aarch64-linux-gnu/libGLdispatch.so.0',

    # 2. CUDA Runtime 라이브러리 (GPU 사용 시 필수)
    '/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so',

    # 3. 추가적인 OpenGL 라이브러리들 (TLS 경쟁 방지)
    #    실제 경로를 ldconfig -p로 확인하여 필요에 따라 추가합니다.
    '/usr/lib/aarch64-linux-gnu/libEGL.so.1',
    '/usr/lib/aarch64-linux-gnu/libGLX.so.0',
    # '/usr/lib/aarch64-linux-gnu/libGL.so.1', # 필요에 따라 추가
]
    ld_preload_path = ":".join(ld_preload_libs)
    set_ld_preload = SetEnvironmentVariable('LD_PRELOAD', ld_preload_path)
    

    robot_ip = os.getenv('ROBOT_IP', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    rviz_config = "cyclonedds_config.rviz"
    
    camera_name = 'front_realsense_camera'
    camera_frame_name = camera_name + '_link'
    camera_info_path = LaunchConfiguration('camera_info_path', default=os.path.join(
        get_package_share_directory('go2_robot'),  # YAML 파일이 있는 패키지 이름
        'config',                                  # config 폴더
        'realsense_camera_params.yaml'             # YAML 파일 이름
    ))
    

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
        # parameters=[{'robot_ip': robot_ip, "conn_type": conn_type}],
    )
    
    go2_realsense_node = Node(
        package='go2_robot',
        executable='go2_realsense_node',
        parameters=[{
            'serial_number': serial_no,
            'camera_name': 'realsense_camera',
            'width': 1280,
            'height': 720,
            'fps': 30.0,
            'show_preview': False,
            'camera_info_path': camera_info_path,
            'camera_frame': camera_frame_name,
            'lidar_frame': 'odom',
            'enable_yolo': True,
            'yolo_model_path': yolo_model_path
        }],
    )
    
    go2_sport_ctrl_node = Node(
        package='go2_robot',
        executable='go2_sport_ctrl_node',
        # parameters=[{'robot_ip': robot_ip, "conn_type": conn_type}],
    )
    
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )
    
    realsense_camera_node = IncludeLaunchDescription(
        # Python launch 파일임을 명시
        PythonLaunchDescriptionSource(realsense_launch_path),
        
        # 포함되는 launch 파일(rs_launch.py)에 전달할 파라미터 설정
        launch_arguments={
            'camera_name': camera_name,
            'serial_no': serial_no,
            'config_file': config_file_path,
            # rs_launch.py에 정의된 다른 파라미터들도 여기서 설정 가능
            # 'pointcloud.enable': 'true',
            # 'align_depth.enable': 'true',
        }.items()
    )
    
    yrlidar_launch_path = os.path.join(
        get_package_share_directory('yrl3_v2_ros2_package'),
        'launch',
        'yrl3_v2_ros2.launch.py'
    )

    yrlidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yrlidar_launch_path),
        # launch_arguments={}.items()
    )

    go2_realsense_transformer_node = Node(
        package='go2_robot',
        executable='go2_realsense_transformer_node',
        name='realsense_transformer_node',
        parameters=[{
            'topic_name': camera_name,
            'camera_info_path': camera_info_path,
            'camera_frame': camera_frame_name,
            'fov_range': 3.0,  # 카메라의 시야 범위 (미터 단위)
        }],
        # env={'LD_PRELOAD': libgomp_path}
    )
    
    go2_image_lidar_overlay_node = Node(
        package='go2_robot',
        executable='go2_image_lidar_overlay_node',
        name='go2_image_lidar_overlay_node',
        parameters=[{
            'topic_name': camera_name,
            'camera_frame': camera_frame_name,
            # 'lidar_name': 'fused_point_cloud',
            'camera_info_path': camera_info_path,
            'fov_range': 2.0,  # 카메라의 시야 범위 (미터 단위)
            'sync_threshold_sec': 0.1,  # 동기화 임계값 (초 단위)
            'yolo_model_path': yolo_model_path,
        }],
        # env={'ORT_DISABLE_AFFINITY': '1'}
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
            'min_height': -0.05,         # 바닥면 노이즈 제거를 위해 추가
            'max_height': 0.05,          # 천장이나 너무 높은 장애물 제거
            'range_min': 0.1,          # 로봇에 너무 가까운 포인트(노이즈) 제거
            'range_max': 10.0,          # 최대 감지 거리 설정
            # 'use_inf': True,            # 측정 범위를 벗어난 거리를 무한대(inf)로 처리cc
        }],
        output='screen',
    )
    
    # --- CuPy 가속화 PointCloud to LaserScan 노드 ---
    cupy_pointcloud_to_laserscan_node = Node(
        package='go2_robot', # 스크립트가 포함된 패키지
        executable='cupy_pointcloud_to_laserscan', # 실행 파일 이름
        name='pointcloud_to_laserscan', # C++ 노드와 동일한 이름 사용
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed'), 
            ('scan', '/scan'), 
        ],
        parameters=[{
            'target_frame': 'base_link', 
            'min_height': -0.1,
            'max_height': 0.1,
            'range_min': 0.1,
            'range_max': 10.0,
            # SLAM 파라미터와 일치시키는 것이 중요합니다.
            'angle_min': -3.1415926535, # -pi
            'angle_max': 3.1415926535,  # +pi
            'angle_increment': 0.0174532925, # 1도 (또는 slam_toolbox 설정값)
            'use_inf': True,
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
        # remappings=[('/odom', '/utlidar/robot_odom')],  # PointCloud to LaserScan 노드의 출력 토픽과 일치시킴
    )
    
    # --- RViz2 노드 ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(with_rviz2),
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('go2_robot'), 'config', rviz_config)]
    )
    go2_realsense_node = Node(
        package='go2_robot',
        executable='realsense_node',
        name='realsense_node',
        parameters=[{
            'serial_number': serial_no,
            'camera_name': camera_name
        }]
    )

    return LaunchDescription([
        set_ld_preload,
        # declare_slam_params_file_cmd,
        # robot_state_publisher_node,
        # go2_driver_node,
        # go2_realsense_node,
        # go2_sport_ctrl_node,
        # go2_image_lidar_overlay_node,
        # go2_realsense_transformer_node,
        # yrlidar_node,
        # pointcloud_to_laserscan_node,
        cupy_pointcloud_to_laserscan_node,
        # start_async_slam_toolbox_node,
        # rviz2_node,
        # go2_realsense_node
    ])