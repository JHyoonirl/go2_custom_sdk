import json
import logging
import os
import threading

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import LowState, IMUState
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy, Imu
# from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_custom_node')

        qos_profile = QoSProfile(depth=100)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.joint_pub = []
        self.go2_state_pub = []
        self.go2_lidar_pub = []
        self.go2_odometry_pub = []
        self.imu_pub = []
        self.img_pub = []
        self.camera_info_pub = []
        self.voxel_pub = []
        
        self.joint_pub.append(self.create_publisher(
            JointState, 'joint_states', qos_profile))
        
        self.go2_state_pub.append(self.create_publisher(
                Go2State, 'go2_states', qos_profile))
        
        self.go2_lidar_pub.append(
                self.create_publisher(
                    PointCloud2,
                    'point_cloud2',
                    best_effort_qos))
        
        self.go2_imu_pubblisher = self.create_publisher(Imu, 'robot_imu', qos_profile)
        
        
        # self.imu_pub.append(self.create_publisher(IMU, 'imu', qos_profile))

        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.create_subscription(
            LowState,
            'lowstate',
            self.publish_joint_state_cyclonedds,
            qos_profile)

        # self.create_subscription(
        #     PoseStamped,
        #     '/utlidar/robot_pose',
        #     self.publish_body_poss_cyclonedds,
        #     qos_profile)
        
        self.last_pose = None
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_cb, qos_profile)
        self.create_timer(0.05, self.publish_body_poss_cyclonedds)  # 20Hz

        self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.publish_lidar_cyclonedds,
            qos_profile)
        
    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg.pose
        
    def publish_body_poss_cyclonedds(self):
        if self.last_pose is None:
            return
    
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = self.last_pose.position.x
        odom_trans.transform.translation.y = self.last_pose.position.y
        odom_trans.transform.translation.z = self.last_pose.position.z + 0.07
        odom_trans.transform.rotation = self.last_pose.orientation
        
        self.broadcaster.sendTransform(odom_trans)

    def publish_joint_state_cyclonedds(self, msg:LowState):
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
        
        # go2_imu = Imu()
        
        # go2_imu.orientation.w = msg.imu_state.quaternion[0]
        
        # go2_imu.orientation.x = msg.imu_state.quaternion[1]
        # go2_imu.orientation.y = msg.imu_state.quaternion[2]
        # go2_imu.orientation.z = msg.imu_state.quaternion[3]
        
        # self.go2_imu_pubblisher.publish(go2_imu)
        
        
        

    def publish_lidar_cyclonedds(self, msg: PointCloud2):
        
        msg.header = Header(frame_id="radar")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[0].publish(msg)



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