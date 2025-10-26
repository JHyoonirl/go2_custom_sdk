import torch
import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cupy as cp
import cv2
import yaml
import math
import tf2_ros
import numpy as np
import traceback
from collections import deque
from scipy.spatial.transform import Rotation
# from cupy.scipy.ndimage import map_coordinates # 구버전 CuPy와 호환성을 위해 제거

# ==============================================================================
# PointCloud2 메시지 처리를 위한 헬퍼 함수
# ==============================================================================

def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    """
    sensor_msgs/PointCloud2 메시지를 (N, 3) 형태의 NumPy 배열로 변환합니다.
    """
    fields_offsets = {field.name: field.offset for field in cloud_msg.fields}
    if 'x' not in fields_offsets or 'y' not in fields_offsets or 'z' not in fields_offsets:
        raise ValueError("PointCloud2 message must have 'x', 'y', and 'z' fields.")

    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0:
        return np.array([], dtype=np.float32).reshape(0, 3)
        
    data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(num_points, cloud_msg.point_step)

    x_offset, y_offset, z_offset = fields_offsets['x'], fields_offsets['y'], fields_offsets['z']
    
    x = data[:, x_offset:x_offset+4].view(np.float32)
    y = data[:, y_offset:y_offset+4].view(np.float32)
    z = data[:, z_offset:z_offset+4].view(np.float32)

    points = np.hstack([x, y, z])

    if remove_nans:
        points = points[~np.isnan(points).any(axis=1)]

    return points

def create_cloud_xyz_rgb(header, points_xyz, colors_rgb):
    """
    (N, 3) XYZ 포인트 배열과 (N, 3) RGB 색상 배열로부터
    색상 정보가 포함된 PointCloud2 메시지를 생성합니다.
    """
    r, g, b = colors_rgb[:, 0].astype(np.uint32), colors_rgb[:, 1].astype(np.uint32), colors_rgb[:, 2].astype(np.uint32)
    rgb_packed = np.bitwise_or(np.left_shift(r, 16), np.bitwise_or(np.left_shift(g, 8), b))
    rgb_packed = rgb_packed.astype(np.float32).view(np.uint32)

    points_with_rgb = np.zeros(points_xyz.shape[0], dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)
    ])
    points_with_rgb['x'], points_with_rgb['y'], points_with_rgb['z'], points_with_rgb['rgb'] = \
        points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2], rgb_packed

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    msg = PointCloud2(
        header=header, height=1, width=len(points_with_rgb),
        is_dense=False, is_bigendian=False, fields=fields,
        point_step=16, row_step=16 * len(points_with_rgb),
        data=points_with_rgb.tobytes()
    )
    return msg

# ==============================================================================
# CuPy를 사용한 이미지 왜곡 보정 헬퍼 함수 (수동 구현)
# ==============================================================================
def remap_cupy_manual(image_gpu, map1_gpu, map2_gpu):
    """
    [수정됨] cupy.scipy 없이 기본 CuPy 연산만으로 Bilinear Interpolation을 사용한
    이미지 왜곡 보정(remap)을 수행하여 구버전 CuPy와 호환성을 확보합니다.
    """
    h, w, c = image_gpu.shape

    # 1. 경계 클리핑: 좌표가 이미지 범위를 벗어나지 않도록 제한
    map1_gpu = cp.clip(map1_gpu, 0, w - 1)
    map2_gpu = cp.clip(map2_gpu, 0, h - 1)

    # 2. 4개의 주변 정수 좌표 계산
    x0 = cp.floor(map1_gpu).astype(cp.int32)
    y0 = cp.floor(map2_gpu).astype(cp.int32)
    
    # x1, y1 좌표도 경계 내에 있도록 다시 클리핑
    x1 = cp.clip(x0 + 1, 0, w - 1)
    y1 = cp.clip(y0 + 1, 0, h - 1)

    # 3. 4개 지점의 픽셀 값 가져오기 (고급 인덱싱)
    Ia = image_gpu[y0, x0]  # Top-left
    Ib = image_gpu[y1, x0]  # Bottom-left
    Ic = image_gpu[y0, x1]  # Top-right
    Id = image_gpu[y1, x1]  # Bottom-right
    
    # 4. 보간 가중치 계산
    wa = (x1 - map1_gpu) * (y1 - map2_gpu)
    wb = (x1 - map1_gpu) * (map2_gpu - y0)
    wc = (map1_gpu - x0) * (y1 - map2_gpu)
    wd = (map1_gpu - x0) * (map2_gpu - y0)

    # 5. 가중치를 채널 차원에 맞게 확장 (H, W) -> (H, W, 1)
    wa, wb, wc, wd = wa[..., None], wb[..., None], wc[..., None], wd[..., None]
    
    # 6. 가중 평균 계산
    remapped_image = wa * Ia + wb * Ib + wc * Ic + wd * Id
    
    return remapped_image.astype(image_gpu.dtype)

# ==============================================================================
# 메인 노드 클래스
# ==============================================================================

class RealsenseTransformer(Node):
    def __init__(self):
        super().__init__('realsense_transformer_cupy')

        # GPU 사용 가능 여부 확인
        try:
            cp.cuda.runtime.getDeviceCount()
            self.gpu_enabled = True
            self.get_logger().info("CuPy 확인 완료! GPU 가속 모드로 실행됩니다.")
        except cp.cuda.runtime.CUDARuntimeError:
            self.gpu_enabled = False
            self.get_logger().info("CuPy를 사용할 수 없습니다. CPU 모드로 실행됩니다.")

        # 파라미터 선언 및 가져오기 (기존과 동일)
        self.declare_parameter('topic_name', 'camera')
        self.declare_parameter('camera_info_path', '')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('lidar_frame', 'odom')
        self.declare_parameter('fov_range', 5.0)

        self.camera_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.camera_info_path = self.get_parameter('camera_info_path').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.range = self.get_parameter('fov_range').get_parameter_value().double_value

        # 멤버 변수 초기화
        self.bridge = CvBridge()
        self.cam_info = None
        self.camera_matrix = None # CPU
        self.dist_coeffs = None   # CPU
        self.lidar_to_cam_matrix = None # CPU

        if self.gpu_enabled:
            # CuPy 스트림 초기화
            self.cp_stream = cp.cuda.Stream()
            # GPU 변수 초기화
            self.lidar_to_cam_matrix_gpu = None
            self.camera_matrix_gpu = None
            self.map1_gpu = None # 왜곡 보정 맵
            self.map2_gpu = None # 왜곡 보정 맵
            
        self._load_camera_info()

        # TF, Publisher, Subscriber 설정 (기존과 거의 동일)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.fused_pub = self.create_publisher(PointCloud2, 'fused_point_cloud', 10)
        self.marker_pub = self.create_publisher(Marker, 'camera_fov_marker', 10)
        self.lidar_buffer = deque(maxlen=200)
        self.last_image_time_sec = None
        custom_lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )
        self.image_sub = self.create_subscription(
            Image, f'/camera/{self.camera_name}/color/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud_deskewed', self.lidar_callback, qos_profile=custom_lidar_qos)
        self.lidar_sub = self.create_subscription( # For yrlidar
            PointCloud2, '/yrl_scan', self.lidar_callback, qos_profile=custom_lidar_qos)
        self.tf_timer = self.create_timer(0.05, self.update_transform_matrix) # 20Hz

        self.get_logger().info('CuPy-based Realsense transformer node has been initialized.')

    def _load_camera_info(self):
        if not self.camera_info_path:
            self.get_logger().error("Camera info path is not provided! Shutting down.")
            rclpy.shutdown(); sys.exit(1)
        try:
            with open(self.camera_info_path, 'r') as file:
                cam_params = yaml.safe_load(file)
                self.cam_info = CameraInfo()
                self.cam_info.width = cam_params['imageSize']['width']
                self.cam_info.height = cam_params['imageSize']['height']
                
                self.camera_matrix = np.array(cam_params['cameraMatrix']['data'], dtype=np.float32).reshape(3, 3)
                self.dist_coeffs = np.array(cam_params['distCoeffs']['data'], dtype=np.float32)
                
                if self.gpu_enabled:
                    # 카메라 행렬 GPU 업로드
                    self.camera_matrix_gpu = cp.asarray(self.camera_matrix)
                    
                    # 왜곡 보정 맵을 CPU에서 계산
                    self.get_logger().info("Calculating undistortion map on CPU...")
                    map1_np, map2_np = cv2.initUndistortRectifyMap(
                        self.camera_matrix, self.dist_coeffs, None, self.camera_matrix,
                        (self.cam_info.width, self.cam_info.height), cv2.CV_32FC1
                    )
                    
                    # 계산된 맵을 GPU로 업로드
                    self.map1_gpu = cp.asarray(map1_np)
                    self.map2_gpu = cp.asarray(map2_np)
                    self.get_logger().info("Undistortion map calculated and uploaded to GPU.")

        except Exception as e:
            self.get_logger().error(f"Failed to load or parse camera info: {e}. Shutting down.")
            rclpy.shutdown(); sys.exit(1)

    def update_transform_matrix(self):
        try:
            trans_stamped = self.tf_buffer.lookup_transform(
                self.camera_frame, self.lidar_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            t = trans_stamped.transform.translation
            translation = np.array([t.x, t.y, t.z])
            q = trans_stamped.transform.rotation
            rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

            transform_matrix = np.eye(4, dtype=np.float32)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = translation
            
            self.lidar_to_cam_matrix = transform_matrix
            
            if self.gpu_enabled:
                self.lidar_to_cam_matrix_gpu = cp.asarray(transform_matrix)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.lidar_frame} to {self.camera_frame}: {ex}', throttle_duration_sec=1.0)

    # image_callback과 lidar_callback은 기존 코드와 동일하여 생략
    def lidar_callback(self, msg):
        self.lidar_buffer.append(msg)

    def image_callback(self, image_msg):
        if not self.lidar_buffer:
            self.get_logger().warn("Lidar buffer is empty, skipping fusion.", throttle_duration_sec=1.0)
            return

        current_image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        
        MAX_LIDAR_AGE_SEC = 2.0
        while self.lidar_buffer:
            oldest_lidar_time = self.lidar_buffer[0].header.stamp.sec + self.lidar_buffer[0].header.stamp.nanosec * 1e-9
            if current_image_time - oldest_lidar_time > MAX_LIDAR_AGE_SEC:
                self.lidar_buffer.popleft()
            else:
                break

        if self.last_image_time_sec is None:
            self.last_image_time_sec = current_image_time
            return

        lidar_msgs_to_process = []
        # 버퍼를 복사해서 순회하며 원본을 수정
        for lidar_msg in list(self.lidar_buffer):
            lidar_time = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9
            if self.last_image_time_sec < lidar_time <= current_image_time:
                lidar_msgs_to_process.append(lidar_msg)
            elif lidar_time <= self.last_image_time_sec:
                 self.lidar_buffer.remove(lidar_msg)

        if lidar_msgs_to_process:
            if self.gpu_enabled:
                self.fuse_data_gpu(image_msg, lidar_msgs_to_process)
            else:
                self.fuse_data_numpy(image_msg, lidar_msgs_to_process)

        self.last_image_time_sec = current_image_time

    # fuse_data_numpy는 기존 코드와 동일하여 생략
    def fuse_data_numpy(self, image_msg, lidar_msgs):
        if self.lidar_to_cam_matrix is None: return
        all_points_list = [pointcloud2_to_xyz_array(msg) for msg in lidar_msgs]
        if not any(lst.size > 0 for lst in all_points_list): return
        points_lidar_frame = np.vstack(all_points_list)

        points_lidar_homogeneous = np.hstack((points_lidar_frame, np.ones((points_lidar_frame.shape[0], 1))))
        points_cam_homogeneous = (self.lidar_to_cam_matrix @ points_lidar_homogeneous.T).T
        
        in_front_mask = points_cam_homogeneous[:, 2] > 0.1
        points_cam_frame = points_cam_homogeneous[in_front_mask, :3]
        original_points_filtered = points_lidar_frame[in_front_mask]
        if points_cam_frame.shape[0] == 0: return

        projected_points = (self.camera_matrix @ points_cam_frame.T).T
        px = projected_points[:, 0] / (projected_points[:, 2] + 1e-6)
        py = projected_points[:, 1] / (projected_points[:, 2] + 1e-6)

        width, height = self.cam_info.width, self.cam_info.height
        bounds_mask = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        valid_px = px[bounds_mask].astype(int)
        valid_py = py[bounds_mask].astype(int)
        final_points_lidar = original_points_filtered[bounds_mask]
        if final_points_lidar.shape[0] == 0: return

        cv_image_numpy = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        undistorted_image = cv2.undistort(cv_image_numpy, self.camera_matrix, self.dist_coeffs)
        
        colors_bgr = undistorted_image[valid_py, valid_px]
        colors_rgb = colors_bgr[:, ::-1]

        header = image_msg.header
        header.frame_id = self.lidar_frame
        fused_cloud_msg = create_cloud_xyz_rgb(header, final_points_lidar, colors_rgb)
        self.fused_pub.publish(fused_cloud_msg)
        self.publish_fov_line_marker()

    def fuse_data_gpu(self, image_msg, lidar_msgs):
        """[핵심 수정] CuPy를 사용하여 GPU에서 융합을 수행합니다."""
        if self.lidar_to_cam_matrix_gpu is None: return

        points_lidar_frame_cpu = np.vstack([pointcloud2_to_xyz_array(msg) for msg in lidar_msgs]).astype(np.float32)
        if points_lidar_frame_cpu.shape[0] == 0: return

        try:
            cv_image_numpy = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # CuPy 스트림 컨텍스트 내에서 비동기 작업 수행
            with self.cp_stream:
                # 1. [CPU -> GPU] 데이터 비동기 업로드
                image_gpu = cp.asarray(cv_image_numpy)
                points_lidar_gpu = cp.asarray(points_lidar_frame_cpu)
                
                # 2. [GPU] 이미지 왜곡 보정 (수동 CuPy Remap 함수 호출)
                undistorted_image_gpu = remap_cupy_manual(image_gpu, self.map1_gpu, self.map2_gpu)
                
                # 3. [GPU] 벡터화된 좌표 변환 (CuPy)
                points_lidar_homogeneous = cp.hstack((
                    points_lidar_gpu, cp.ones((points_lidar_gpu.shape[0], 1), dtype=cp.float32)
                ))
                points_cam_homogeneous = (self.lidar_to_cam_matrix_gpu @ points_lidar_homogeneous.T).T
                
                # 이후 로직은 기존과 동일 (변수명만 _gpu로)
                in_front_mask = points_cam_homogeneous[:, 2] > 0.1
                points_cam_frame = points_cam_homogeneous[in_front_mask, :3]
                original_points_filtered_gpu = points_lidar_gpu[in_front_mask]
                if points_cam_frame.shape[0] == 0: return

                projected_points = (self.camera_matrix_gpu @ points_cam_frame.T).T
                px = projected_points[:, 0] / (projected_points[:, 2] + 1e-6)
                py = projected_points[:, 1] / (projected_points[:, 2] + 1e-6)

                width, height = self.cam_info.width, self.cam_info.height
                bounds_mask = (px >= 0) & (px < width) & (py >= 0) & (py < height)
                
                valid_px_gpu = px[bounds_mask].astype(cp.int32)
                valid_py_gpu = py[bounds_mask].astype(cp.int32)
                final_points_lidar_gpu = original_points_filtered_gpu[bounds_mask]
                
                # 4. [GPU -> CPU] 최종 데이터 다운로드
                undistorted_image_cpu = undistorted_image_gpu.get()
                valid_px_cpu = valid_px_gpu.get()
                valid_py_cpu = valid_py_gpu.get()
                final_points_lidar_cpu = final_points_lidar_gpu.get()
            
            # 스트림 동기화 (모든 GPU 작업이 끝날 때까지 대기)
            self.cp_stream.synchronize()

            # 5. [CPU] 색상 추출 및 메시지 생성/발행
            if final_points_lidar_cpu.shape[0] == 0: return
            colors_bgr = undistorted_image_cpu[valid_py_cpu, valid_px_cpu]
            colors_rgb = colors_bgr[:, ::-1] # BGR -> RGB

            header = image_msg.header
            header.frame_id = self.lidar_frame
            fused_cloud_msg = create_cloud_xyz_rgb(header, final_points_lidar_cpu, colors_rgb)
            self.fused_pub.publish(fused_cloud_msg)
            self.publish_fov_line_marker()

        except Exception as e:
            self.get_logger().error(f"GPU FUSION 중 오류 발생: {e}\n{traceback.format_exc()}", throttle_duration_sec=1.0)

    # Marker 및 FOV 계산 함수는 기존 코드와 동일하여 생략
    def publish_fov_line_marker(self):
        fov_x, fov_y = self.calculate_fov_from_camera()
        if fov_x is None: return

        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fov_lines"; marker.id = 1
        marker.type = Marker.LINE_LIST; marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.8

        z = self.range
        x = z * math.tan(fov_x / 2.0)
        y = z * math.tan(fov_y / 2.0)
        
        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=-x, y=-y, z=z); p2 = Point(x=x, y=-y, z=z)
        p3 = Point(x=x, y=y, z=z); p4 = Point(x=-x, y=y, z=z)

        marker.points = [p0, p1, p0, p2, p0, p3, p0, p4, p1, p2, p2, p3, p3, p4, p4, p1]
        self.marker_pub.publish(marker)

    def calculate_fov_from_camera(self):
        if self.camera_matrix is None: return None, None
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        fov_x = 2 * math.atan(self.cam_info.width / (2 * fx))
        fov_y = 2 * math.atan(self.cam_info.height / (2 * fy))
        return fov_x, fov_y

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

