import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
import threading
import queue

class RealSensePublisher(Node):
    """
    지정된 시리얼 번호의 RealSense 카메라에서 이미지를 캡처하여 ROS 2 토픽으로 발행합니다.
    [수정] 프레임 획득을 별도 스레드로 분리하여 시스템 부하 시에도 FPS를 안정적으로 유지합니다.
    """
    def __init__(self):
        super().__init__('realsense_publisher')

        # --- ROS 2 파라미터 선언 ---
        self.declare_parameter('serial_number', '233722072808') # 기본값은 없는 시리얼로 설정
        self.declare_parameter('camera_name', 'realsense_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('show_preview', False)

        # --- 파라미터 값 가져오기 ---
        self.serial_number = self.get_parameter('serial_number').get_parameter_value().string_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value

        if self.serial_number == '000000000000':
            self.get_logger().warn("'serial_number' 파라미터가 설정되지 않았습니다. 연결된 첫 번째 RealSense 장치를 사용합니다.")
        
        # --- QoS 프로파일 설정 (센서 데이터에 적합) ---
        self.qos_profile = QoSProfile(
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            # durability=QoSDurabilityPolicy.VOLATILE,
            # history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            Image,
            f'/camera/{self.camera_name}/color/image_raw',
            self.qos_profile
        )

        self.bridge = CvBridge()

        # --- RealSense 파이프라인 설정 ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if self.serial_number != '000000000000':
            self.config.enable_device(self.serial_number)

        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, int(self.fps))

        try:
            self.pipeline.start(self.config)
            self.get_logger().info(
                f'RealSense 파이프라인 시작 ({self.camera_name}): S/N={self.serial_number or "any"}'
            )
            self.get_logger().info(f'요청: {self.width}x{self.height} @ {self.fps}Hz')
        except Exception as e:
            self.get_logger().error(f'RealSense 파이프라인 시작 실패: {e}')
            rclpy.shutdown()
            return

        # --- 스레드 기반 프레임 읽기 설정 ---
        self.frame_queue = queue.Queue(maxsize=1)
        self.is_running = True
        self.frame_reader_thread = threading.Thread(target=self._frame_reader_loop)
        self.frame_reader_thread.daemon = True
        self.frame_reader_thread.start()

        # --- ROS 타이머를 사용하여 퍼블리싱 ---
        if self.fps > 0:
            timer_period = 1.0 / self.fps
            self.publish_timer = self.create_timer(timer_period, self._publish_callback)

    def _frame_reader_loop(self):
        """RealSense 카메라에서 프레임을 계속 읽어 큐에 넣는 스레드 함수."""
        while self.is_running and rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                if not frames:
                    continue
                
                color_frame = frames.get_color_frame()
                if color_frame:
                    frame_data = np.asanyarray(color_frame.get_data())
                    # 큐가 비어있으면 넣고, 꽉 차있으면 기존 것을 버리고 새 것을 넣음
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self.frame_queue.put(frame_data)
            except RuntimeError as e:
                self.get_logger().warn(f"프레임 획득 중 오류: {e}")


    def _publish_callback(self):
        """타이머에 의해 호출되어 큐에서 프레임을 꺼내 발행하는 함수."""
        try:
            frame = self.frame_queue.get_nowait()
        except queue.Empty:
            return

        now = self.get_clock().now()
        
        # --- ROS 메시지 발행 로직 ---
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = now.to_msg()
        image_msg.header.frame_id = f"{self.camera_name}_optical_frame"
        self.publisher_.publish(image_msg)

        if self.show_preview:
            cv2.imshow(f"RealSense Stream: {self.camera_name}", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """ 노드 종료 시 스레드를 중지하고 파이프라인과 OpenCV 창을 안전하게 닫습니다. """
        self.get_logger().info(f'RealSense 퍼블리셔({self.camera_name}) 종료 중...')
        self.is_running = False
        if hasattr(self, 'frame_reader_thread') and self.frame_reader_thread.is_alive():
            self.frame_reader_thread.join()
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
        if self.show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    realsense_publisher = RealSensePublisher()
    try:
        rclpy.spin(realsense_publisher)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        realsense_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()