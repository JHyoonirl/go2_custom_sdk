
import torch
import os
# [수정] ONNX Runtime의 CPU 스레드 관련 에러를 방지하기 위해 환경 변수를 코드 내에서 직접 설정합니다.
# 다른 라이브러리가 로드되기 전에 가장 먼저 실행되어야 합니다.
# os.environ['ORT_SESSION_THREAD_COUNT'] = '1'

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime


from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped

import tf2_ros
from cv_bridge import CvBridge
import numpy as np
import struct
import yaml
import copy
import traceback
from scipy.spatial.transform import Rotation
import cv2 # Standard OpenCV is used for image I/O and CPU undistortion
import os 

# ==============================================================================
# CuPy 설정 및 가져오기
# ==============================================================================
try:
    import cupy as cp
    
    _CUPY_ENABLED = True
except ImportError:
    _CUPY_ENABLED = False
    
try:
    from ultralytics import YOLO
    _YOLO_ENABLED = True
except ImportError:
    _YOLO_ENABLED = False

if not _CUPY_ENABLED:
    print("="*50)
    print("WARNING: CuPy 모듈을 찾을 수 없습니다.")
    print("NumPy 기반 CPU 모드로 대체합니다.")
    print("="*50)

# ==============================================================================
# PointCloud2 메시지 처리를 위한 헬퍼 함수
# (이 함수는 CPU NumPy를 사용하여 데이터를 파싱합니다)
# ==============================================================================

# PointField 데이터 타입과 NumPy dtype 간의 매핑
DTYPE_MAP = {
    PointField.INT8: np.int8, PointField.UINT8: np.uint8,
    PointField.INT16: np.int16, PointField.UINT16: np.uint16,
    PointField.INT32: np.int32, PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64
}


def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    """
    sensor_msgs/PointCloud2 메시지를 (N, 3) XYZ NumPy 배열로 변환합니다.
    (색상 정보는 사용하지 않으므로 XYZ만 추출합니다.)
    """
    fields_offsets = {field.name: field.offset for field in cloud_msg.fields}
    
    required_fields = ['x', 'y', 'z']
    if not all(f in fields_offsets for f in required_fields):
        raise ValueError("PointCloud2 message must have 'x', 'y', and 'z' fields.")

    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0:
        return np.array([]).reshape(0, 3)
        
    data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(num_points, cloud_msg.point_step)

    # XYZ 추출
    x_offset, y_offset, z_offset = fields_offsets['x'], fields_offsets['y'], fields_offsets['z']
    
    x = data[:, x_offset:x_offset+4].view(np.float32)
    y = data[:, y_offset:y_offset+4].view(np.float32)
    z = data[:, z_offset:z_offset+4].view(np.float32)
    
    # [수정] np.vstack(...).T 대신 더 안정적인 np.column_stack 사용
    points_xyz = np.column_stack((x, y, z))

    if remove_nans:
        valid_mask = ~np.isnan(points_xyz).any(axis=1)
        points_xyz = points_xyz[valid_mask]

    return points_xyz


# ==============================================================================
# 메인 오버레이 노드 클래스
# ==============================================================================

class PointCloudOverlay(Node):
    """
    CuPy를 사용하여 융합된 포인트 클라우드를 이미지에 오버레이합니다.
    """
    def __init__(self):
        super().__init__('pointcloud_overlay_node_cupy_only')
        
        # 파라미터 선언 및 가져오기
        self.declare_parameter('topic_name', 'front_realsense_camera')
        self.declare_parameter('camera_info_path', '../config/realsense_camera_params.yaml')
        self.declare_parameter('camera_frame', 'front_realsense_camera_link')
        self.declare_parameter('lidar_frame', 'odom')
        self.declare_parameter('yolo_model_path', '../yolo_models/yolov8s.engine')

        self.camera_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.camera_info_path = self.get_parameter('camera_info_path').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value

        # 멤버 변수 초기화
        self.bridge = CvBridge()
        self.cam_info_loaded = False
        self.width, self.height = 0, 0
        
        # CPU variables
        self.camera_matrix_cpu = None
        self.dist_coeffs_cpu = None

        # [수정] Z축 깊이에 따른 색상 매핑을 위한 최소/최대 Z값 (미터 단위)
        self.min_z_color = 0.0  # 0.1미터보다 가까운 점은 min 색상으로 표시
        self.max_z_color = 5.0 # 10미터보다 먼 점은 max 색상으로 표시

        # CuPy setup
        self.cupy_enabled = _CUPY_ENABLED
        if self.cupy_enabled:
            self.get_logger().info("CuPy mode enabled. (GPU acceleration for projection)")
            self.cp_stream = cp.cuda.Stream()
            self.camera_matrix_cupy = None # (CuPy 3, 3)
        else:
            self.get_logger().warn("CuPy module not available. Running in pure CPU (NumPy) mode.")
            
        # [수정] YOLO 모델 로딩 로직 수정
        self.yolo_enabled = _YOLO_ENABLED
        self.yolo_model = None # YOLO 모델을 저장할 멤버 변수
        if self.yolo_enabled:
            self.get_logger().info("YOLO module found. Initializing object detection model.")
            self.initialize_yolo()
        else:
            self.get_logger().warn("YOLO (ultralytics) module not found. Object detection will be disabled.")
            
            
        self._load_camera_info()

        # TF 리스너 및 버퍼
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # [수정] 데이터 저장을 위한 변수 변경
        self.last_image_msg = None
        self.cloud_buffer = [] # (timestamp, points_xyz)를 저장하는 리스트

        # 퍼블리셔 및 구독자
        self.fused_image_pub = self.create_publisher(Image, 'fused_image', 10)
        
        self.image_sub = self.create_subscription(
            Image, f'/camera/{self.camera_name}/color/image_raw', self.image_callback, 10
        )
        
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'fused_point_cloud', self.cloud_callback, 10
        )
        
        # 주기적인 처리 타이머
        self.process_timer = self.create_timer(0.075, self.process_data) # 약 20Hz

        self.get_logger().info('CuPy Accelerated PointCloud Overlay node has been initialized.')
        
    def initialize_yolo(self):
        """ [추가] YOLO 모델을 초기화하고 TensorRT 엔진을 로드하는 함수 """
        model_engine_path = self.yolo_model_path
        model_pt_path = self.yolo_model_path

        try:
            # if not os.path.exists(model_engine_path):
            #     self.get_logger().info(f"'{model_engine_path}' not found. Exporting from '{model_pt_path}'...")
            #     if not os.path.exists(model_pt_path):
            #          self.get_logger().error(f"'{model_pt_path}' not found. Please download it first.")
            #          self.yolo_enabled = False
            #          return
            #     model = YOLO(model_pt_path)
            #     self.get_logger().info(f"'found.")
            #     # model.export(format='engine') # TensorRT 엔진 생성
            #     # self.get_logger().info(f"Successfully exported to '{model_engine_path}'.")

            self.get_logger().info(f"Loading TensorRT model from '{model_engine_path}'...")
            self.yolo_model = YOLO(model_engine_path)
            self.get_logger().info("YOLO model loaded successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize YOLO model: {e}")
            self.yolo_enabled = False

    def _load_camera_info(self):
        """YAML 파일로부터 카메라 파라미터를 로드하고 CuPy에 업로드합니다."""
        if not self.camera_info_path:
            self.get_logger().error("Camera info path is not provided! Skipping camera info loading.")
            return

        try:
            with open(self.camera_info_path, 'r') as file:
                cam_params = yaml.safe_load(file)
                
                self.width = cam_params['imageSize']['width']
                self.height = cam_params['imageSize']['height']
                
                self.camera_matrix_cpu = np.array(cam_params['cameraMatrix']['data'], dtype=np.float32).reshape(3, 3)
                self.dist_coeffs_cpu = np.array(cam_params['distCoeffs']['data'], dtype=np.float32)
                self.cam_info_loaded = True
                
                if self.cupy_enabled:
                    self.camera_matrix_cupy = cp.asarray(self.camera_matrix_cpu)
                    self.get_logger().info("Camera matrix uploaded to CuPy/GPU memory.")
                    
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse camera info: {e}. Shutting down.")
            self.cam_info_loaded = False

    def image_callback(self, msg):
        self.last_image_msg = msg

    def cloud_callback(self, msg):
        """수신된 포인트 클라우드를 파싱하여 버퍼에 저장합니다."""
        try:
            points_xyz = pointcloud2_to_xyz_array(msg)
            if points_xyz.shape[0] > 0:
                self.cloud_buffer.append((msg.header.stamp, points_xyz))
        except ValueError as e:
            self.get_logger().warn(f"Could not parse PointCloud2: {e}")

    def process_data(self):
        """최신 이미지와 누적된 포인트 클라우드를 가져와 처리합니다."""
        if not self.cam_info_loaded or self.last_image_msg is None:
            self.get_logger().info("Camera info not loaded or no image received yet.")
            return

        image_msg = copy.deepcopy(self.last_image_msg)
        self.last_image_msg = None

        # [수정] 1초 Decay Time 로직
        current_stamp = RclpyTime.from_msg(image_msg.header.stamp)
        decay_duration = Duration(seconds=1.0)

        self.cloud_buffer = [
            (stamp, points) for stamp, points in self.cloud_buffer 
            if (current_stamp - RclpyTime.from_msg(stamp)) < decay_duration
        ]

        # 버퍼에 포인트가 없으면 이미지만 처리
        if not self.cloud_buffer:
            try:
                cv_image_raw = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                undistorted_image = cv2.undistort(cv_image_raw, self.camera_matrix_cpu, self.dist_coeffs_cpu)
                # [수정] 포인트 클라우드가 없어도 YOLO는 수행
                final_image = self.run_yolo_detection(undistorted_image)
                self.publish_image(final_image, image_msg.header)
            except Exception as e:
                self.get_logger().error(f"Image processing failed with empty buffer: {e}")
            return
            
        # 버퍼의 모든 포인트를 하나로 합칩니다.
        aggregated_points_cpu = np.vstack([points for _, points in self.cloud_buffer])

        if self.cupy_enabled:
            self.process_data_cupy(image_msg, aggregated_points_cpu)
        else:
            self.process_data_numpy(image_msg, aggregated_points_cpu)
            
    def run_yolo_detection(self, image_cpu):
        """ [추가] 이미지에 YOLO 탐지를 수행하고 바운딩 박스를 그리는 함수 """
        if self.yolo_enabled and self.yolo_model:
            # YOLO 모델로 추론 수행
            results = self.yolo_model(image_cpu, verbose=False, classes=0)
            # ultralytics 라이브러리의 plot() 함수로 바운딩 박스를 이미지에 직접 그림
            image_with_boxes = results[0].plot()
            return image_with_boxes
        else:
            # YOLO가 비활성화된 경우 원본 이미지 반환
            return image_cpu

    def _get_transform_matrix(self, transform_stamped: TransformStamped):
        """TransformStamped에서 (4, 4) 동차 변환 행렬을 생성합니다."""
        t = transform_stamped.transform.translation
        translation = np.array([t.x, t.y, t.z])
        q = transform_stamped.transform.rotation
        rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        return transform_matrix

    def process_data_cupy(self, image_msg, points_lidar_frame_cpu):
        """[CuPy] 누적된 포인트 클라우드를 Z축 깊이로 색칠하여 오버레이합니다."""
        try:
            cv_image_raw = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            # self.get_logger().info("Image converted to CV2 format.")
            undistorted_image_cpu = cv2.undistort(cv_image_raw, self.camera_matrix_cpu, self.dist_coeffs_cpu)
            # self.get_logger().info("Image undistorted on CPU.")
            # 2. [추가] YOLO 객체 탐지 수행
            image_with_boxes = self.run_yolo_detection(undistorted_image_cpu)
            # self.get_logger().info("YOLO detection completed on CPU.")
        except Exception as e:
            self.get_logger().error(f"CPU image processing (undistort) failed: {e}")
            return

        try:
            # TF 변환은 이미지 타임스탬프를 기준으로 조회합니다.
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.lidar_frame, image_msg.header.stamp, timeout=Duration(seconds=0.1)
            )
            transform_matrix_cpu = self._get_transform_matrix(transform)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.lidar_frame} to {self.camera_frame}: {ex}', throttle_duration_sec=1.0)
            self.publish_image(image_with_boxes, image_msg.header) # YOLO 결과만이라도 발행
            return
        
        try:
            with self.cp_stream:
                points_lidar_gpu = cp.asarray(points_lidar_frame_cpu.astype(cp.float32))
                transform_matrix_cupy = cp.asarray(transform_matrix_cpu)
                
                points_lidar_homogeneous = cp.hstack((points_lidar_gpu, cp.ones((points_lidar_gpu.shape[0], 1), dtype=cp.float32)))
                points_cam_homogeneous = (transform_matrix_cupy @ points_lidar_homogeneous.T).T
                points_cam_frame = points_cam_homogeneous[:, :3]

                # [수정] Z축 깊이 값을 추출합니다.
                z_values_gpu = points_cam_frame[:, 2]

                projected_points = (self.camera_matrix_cupy @ points_cam_frame.T).T
                Z = projected_points[:, 2]
                
                valid_Z_mask = Z > 0.001 
                px = cp.where(valid_Z_mask, projected_points[:, 0] / Z, cp.nan)
                py = cp.where(valid_Z_mask, projected_points[:, 1] / Z, cp.nan)

                final_mask = (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height) & (~cp.isnan(px))
                
                valid_px_gpu = px[final_mask].astype(cp.int32)
                valid_py_gpu = py[final_mask].astype(cp.int32)
                valid_z_values_gpu = z_values_gpu[final_mask]

                valid_px_cpu = valid_px_gpu.get(stream=self.cp_stream)
                valid_py_cpu = valid_py_gpu.get(stream=self.cp_stream)
                valid_z_values_cpu = valid_z_values_gpu.get(stream=self.cp_stream)

            self.cp_stream.synchronize()

            if len(valid_px_cpu) == 0:
                self.publish_image(undistorted_image_cpu, image_msg.header)
                return

            # 5. [수정] 바운딩 박스가 그려진 이미지 위에 포인트 클라우드 오버레이
            norm_z = np.clip((valid_z_values_cpu - self.min_z_color) / (self.max_z_color - self.min_z_color) * 255, 0, 255)
            norm_z_uint8 = norm_z.astype(np.uint8)
            final_colors_bgr = cv2.applyColorMap(norm_z_uint8.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

            final_overlay_image = image_with_boxes.copy() 
            for i in range(len(valid_px_cpu)):
                color_bgr = tuple(map(int, final_colors_bgr[i]))
                cv2.circle(final_overlay_image, (valid_px_cpu[i], valid_py_cpu[i]), radius=2, color=color_bgr, thickness=-1)
            self.publish_image(final_overlay_image, image_msg.header)

        except Exception as e:
            self.get_logger().error(f"CuPy FUSION 중 오류 발생: {traceback.format_exc()}", throttle_duration_sec=1.0)
            self.publish_image(image_with_boxes, image_msg.header)

    def process_data_numpy(self, image_msg, points_lidar_frame):
        """ [수정] NumPy 파이프라인: YOLO 먼저, LiDAR 오버레이 나중에 """
        try:
            # 1. 이미지 왜곡 보정
            cv_image_raw = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            cv_image_undistorted = cv2.undistort(cv_image_raw, self.camera_matrix_cpu, self.dist_coeffs_cpu)

            # 2. [추가] YOLO 객체 탐지 수행
            image_with_boxes = self.run_yolo_detection(cv_image_undistorted)
        except Exception as e:
            self.get_logger().error(f"CPU image processing (undistort/yolo) failed: {e}")
            return

        try:
            # 3. TF 변환 조회
            transform = self.tf_buffer.lookup_transform(self.camera_frame, self.lidar_frame, image_msg.header.stamp, timeout=Duration(seconds=0.1))
            transform_matrix = self._get_transform_matrix(transform)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.lidar_frame} to {self.camera_frame}: {ex}', throttle_duration_sec=1.0)
            self.publish_image(image_with_boxes, image_msg.header)
            return

        # 4. NumPy를 이용한 포인트 클라우드 투영
        points_lidar_homogeneous = np.hstack((points_lidar_frame, np.ones((points_lidar_frame.shape[0], 1))))
        points_cam_homogeneous = (transform_matrix @ points_lidar_homogeneous.T).T
        points_cam_frame = points_cam_homogeneous[:, :3]
        z_values = points_cam_frame[:, 2]

        projected_points = (self.camera_matrix_cpu @ points_cam_frame.T).T
        Z = projected_points[:, 2]

        valid_Z_mask = Z > 0.001 
        px = np.where(valid_Z_mask, projected_points[:, 0] / Z, np.nan)
        py = np.where(valid_Z_mask, projected_points[:, 1] / Z, np.nan)
        final_mask = (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height) & (~np.isnan(px))
        
        valid_px = px[final_mask].astype(int)
        valid_py = py[final_mask].astype(int)
        valid_z_values = z_values[final_mask]
        
        if valid_px.shape[0] == 0:
            self.publish_image(image_with_boxes, image_msg.header)
            return
        
        # 5. [수정] 바운딩 박스가 그려진 이미지 위에 포인트 클라우드 오버레이
        norm_z = np.clip((valid_z_values - self.min_z_color) / (self.max_z_color - self.min_z_color) * 255, 0, 255)
        norm_z_uint8 = norm_z.astype(np.uint8)
        final_colors_bgr = cv2.applyColorMap(norm_z_uint8.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

        final_overlay_image = image_with_boxes.copy() 
        for i in range(len(valid_px)):
            color_bgr = tuple(map(int, final_colors_bgr[i]))
            cv2.circle(final_overlay_image, (valid_px[i], valid_py[i]), radius=2, color=color_bgr, thickness=-1)
        self.publish_image(final_overlay_image, image_msg.header)

    def publish_image(self, cv_image, original_header):
        """CvImage를 ROS Image 메시지로 변환하여 발행하고, 화면에도 표시합니다."""
        try:
            display_image = cv2.resize(cv_image, (1280, 720))
            cv2.imshow('Fused Image', display_image)
            cv2.waitKey(1)

            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header.stamp = original_header.stamp
            image_msg.header.frame_id = original_header.frame_id
            self.fused_image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"CvBridge failed to convert or publish image: {e}")

# ------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

