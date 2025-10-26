import json
import logging
import os
import threading

import cv2
# from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
# from unitree_go.msg import Go2State
from unitree_go.msg import LowState, IMUState
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy, Imu
# from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

import sys
import os

print("--- Python Environment Debug ---")
# 1. 현재 사용 중인 Python 실행 파일의 절대 경로
print(f"EXECUTABLE: {sys.executable}")

# 2. 파이썬이 라이브러리를 찾는 모든 경로 (sys.path)
print("SYS.PATH:")
for path in sys.path:
    print(f"  - {path}")

# 3. PyTorch를 임포트 해보고, 성공 시 torch 라이브러리 파일 위치 출력
# try:
#     import torch
#     print(f"\nSUCCESSFULLY IMPORTED TORCH!")
#     print(f"  - TORCH VERSION: {torch.__version__}")
#     print(f"  - TORCH FILE: {torch.__file__}")
# except ImportError as e:
#     print(f"\nFAILED TO IMPORT TORCH: {e}")
# print("--------------------------------\n")

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_custom_node')

        qos_profile = QoSProfile(depth=10)
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        
        ## Go2 Publishers ##
        # Go2 URDF sync #
        self.joint_pub = []
        self.go2_state_pub = []
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        
        # Nav2 & SLAM
        # self.go2_lidar_pub = []
        self.go2_odometry_pub = []
        self.imu_pub = []
        
        # Camera
        self.img_pub = []
        self.camera_info_pub = []
        self.voxel_pub = []
        
        
        ## Publishers input ##
        # Go2 URDF sync #
        self.joint_pub.append(self.create_publisher(
            JointState, 'joint_states', qos_profile))
        
        # self.go2_state_pub.append(self.create_publisher(
        #         Go2State, 'go2_states', qos_profile))
        
        # Nav2 & SLAM
        self.go2_odometry_pub.append(self.create_publisher(
            Odometry, 'odometry', qos_profile))        
        self.imu_pub.append(self.create_publisher(
            Imu, 'imu', qos_profile))
        
        # Camera
        self.img_pub.append(self.create_publisher(
            Image, 'image_raw', qos_profile))
        
        timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.camera_callback)
        
        # self.cap = cv2.VideoCapture(1)  # 기본 카메라 장치 열기
        # if not self.cap.isOpened():
        #     self.get_logger().error("카메라를 열 수 없습니다.")
            
        # self.bridge = CvBridge()
        # 1. LowState 메시지를 저장할 변수와 Lock을 추가합니다.
        self.last_lowstate_msg = None
        self.lowstate_lock = threading.Lock()

        self.create_subscription(
            LowState,
            'lowstate',
            self.lowstate_callback,
            qos_profile)

        
        
        self.last_pose = None
        self.last_odom = None
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, qos_profile)
        
        self.create_timer(0.05, self.publish_cyclonedds)  # 20Hz
        
    def lowstate_callback(self, msg: LowState):
        """
        [500Hz 실행] /lowstate 토픽을 받을 때마다 최신 메시지를 저장합니다.
        Lock을 사용하여 타이머와의 충돌을 방지합니다.
        """

        with self.lowstate_lock:
            self.last_lowstate_msg = msg
        
    def publish_cyclonedds(self):
        # self.get_logger().info('CycloneDDS mode activated')
        self.publish_body_poss_cyclonedds()
        self.publish_odometry_cyclonedds()
        self.publish_joint_state_cyclonedds()

    def odom_cb(self, msg: Odometry):
        self.last_pose = msg.pose.pose
        self.last_odom = msg

    def publish_body_poss_cyclonedds(self):
        if self.last_pose is None:
            return
    
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = "base_link"
        
        # translation
        odom_trans.transform.translation.x = self.last_pose.position.x
        odom_trans.transform.translation.y = self.last_pose.position.y
        odom_trans.transform.translation.z = self.last_pose.position.z + 0.035

        # rotation
        odom_trans.transform.rotation.w = self.last_pose.orientation.w
        odom_trans.transform.rotation.x = self.last_pose.orientation.x
        odom_trans.transform.rotation.y = self.last_pose.orientation.y
        odom_trans.transform.rotation.z = self.last_pose.orientation.z

        self.broadcaster.sendTransform(odom_trans)

    def publish_odometry_cyclonedds(self):
        """Publish Odometry topic"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # if self.config.conn_mode == 'single':
        odom_msg.child_frame_id = "base_link"
        # else:
            # odom_msg.child_frame_id = f"robot{robot_data.robot_id}/base_link"
        if self.last_odom is None:
            return

        odom_msg.pose = self.last_odom.pose
        odom_msg.twist = self.last_odom.twist

        self.go2_odometry_pub[0].publish(odom_msg)
    
    def publish_joint_state_cyclonedds(self):
        
        with self.lowstate_lock:
            if self.last_lowstate_msg is None:
                # self.get_logger().warn('No LowState message received yet.')
                return
            # 발행 로직에서 사용할 메시지를 지역 변수로 복사
            msg = self.last_lowstate_msg
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'FL_hip_joint',
            'FL_thigh_joint',
            'FL_calf_joint',
            'FR_hip_joint',
            'FR_thigh_joint',
            'FR_calf_joint',
            'RL_hip_joint',
            'RL_thigh_joint',
            'RL_calf_joint',
            'RR_hip_joint',
            'RR_thigh_joint',
            'RR_calf_joint',
        ]
        joint_state.position = [
            float(msg.motor_state[3].q), float(msg.motor_state[4].q), float(msg.motor_state[5].q),
            float(msg.motor_state[0].q), float(msg.motor_state[1].q), float(msg.motor_state[2].q),
            float(msg.motor_state[9].q), float(msg.motor_state[10].q), float(msg.motor_state[11].q),
            float(msg.motor_state[6].q), float(msg.motor_state[7].q), float(msg.motor_state[8].q),
        ]
        # self.get_logger().info('operating')
        self.joint_pub[0].publish(joint_state)



def main():
    """Main entry point with proper initialization and cleanup."""
    # Initialize ROS
    rclpy.init()

    node = RobotBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()