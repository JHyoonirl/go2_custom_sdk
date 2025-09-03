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
from go2_interfaces.msg import Go2State
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
        
        self.go2_state_pub.append(self.create_publisher(
                Go2State, 'go2_states', qos_profile))
        
        # Nav2 & SLAM


        self.go2_odometry_pub.append(self.create_publisher(
            Odometry, 'odometry', qos_profile))
        
        
        self.imu_pub.append(self.create_publisher(Imu, 'imu', qos_profile))

        self.create_subscription(
            LowState,
            'lowstate',
            self.publish_joint_state_cyclonedds,
            qos_profile)

        
        
        self.last_pose = None
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_cb, qos_profile)
        

        # self.create_subscription(
        #     PointCloud2,
        #     '/utlidar/cloud_deskewed',
        #     self.publish_lidar_cyclonedds,
        #     qos_profile)
        
        # self.create_subscription(
        #     PoseStamped,
        #     '/utlidar/robot_pose',
        #     self.publish_body_poss_cyclonedds,
        #     qos_profile)
        
                
        # self.go2_lidar_pub.append(
        #         self.create_publisher(
        #             PointCloud2,
        #             'point_cloud2',
        #             best_effort_qos))
        
        self.create_timer(0.05, self.publish_body_poss_cyclonedds)  # 20Hz
        
        
    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg.pose
        
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
        odom_trans.transform.translation.z = self.last_pose.position.z + 0.07

        # rotation
        odom_trans.transform.rotation = self.last_pose.orientation
        
        self.broadcaster.sendTransform(odom_trans)

    def publish_odometry_cyclonedds(self, msg: Odometry):
        """Publish Odometry topic"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # if self.config.conn_mode == 'single':
        odom_msg.child_frame_id = "base_link"
        # else:
            # odom_msg.child_frame_id = f"robot{robot_data.robot_id}/base_link"

        position = self.last_pose.position
        orientation = self.last_pose.orientation

        odom_msg.pose.pose.position.x = float(position['x'])
        odom_msg.pose.pose.position.y = float(position['y'])
        odom_msg.pose.pose.position.z = float(position['z']) + 0.07

        odom_msg.pose.pose.orientation.x = float(orientation['x'])
        odom_msg.pose.pose.orientation.y = float(orientation['y'])
        odom_msg.pose.pose.orientation.z = float(orientation['z'])
        odom_msg.pose.pose.orientation.w = float(orientation['w'])
        
        self.go2_odometry_pub[0].publish(odom_msg)

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
        
        go2_imu = Imu()

        go2_imu.orientation.w = float(msg.imu_state.quaternion[0])
        go2_imu.orientation.x = float(msg.imu_state.quaternion[1])
        go2_imu.orientation.y = float(msg.imu_state.quaternion[2])
        go2_imu.orientation.z = float(msg.imu_state.quaternion[3])

        self.imu_pub[0].publish(go2_imu)
        
        
        

    # def publish_lidar_cyclonedds(self, msg: PointCloud2):
        
    #     msg.header = Header(frame_id="radar")
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     self.go2_lidar_pub[0].publish(msg)



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