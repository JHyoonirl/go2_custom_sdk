import torch
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray # 이미 이 메시지를 사용 중일 것입니다.
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import queue
import yaml
import traceback
from scipy.spatial.transform import Rotation
import tf2_ros
from collections import deque
import struct

# ==============================================================================
# YOLO 및 CuPy 라이브러리 임포트 (선택적)
# ==============================================================================
try:
    from ultralytics import YOLO
    _YOLO_ENABLED = True
except ImportError:
    _YOLO_ENABLED = False

try:
    import cupy as cp
    _CUPY_ENABLED = True
except ImportError:
    _CUPY_ENABLED = False

# ==============================================================================
# 헬퍼 함수
# ==============================================================================

def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    # ... (기존과 동일)
    fields_offsets = {field.name: field.offset for field in cloud_msg.fields}
    if not all(f in fields_offsets for f in ['x', 'y', 'z']):
        raise ValueError("PointCloud2 message must have 'x', 'y', and 'z' fields.")

    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0:
        return np.array([], dtype=np.float32).reshape(0, 3)

    data = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(num_points, cloud_msg.point_step)
    x = data[:, fields_offsets['x']:fields_offsets['x']+4].view(np.float32)
    y = data[:, fields_offsets['y']:fields_offsets['y']+4].view(np.float32)
    z = data[:, fields_offsets['z']:fields_offsets['z']+4].view(np.float32)
    points_xyz = np.column_stack((x, y, z))

    if remove_nans:
        points_xyz = points_xyz[~np.isnan(points_xyz).any(axis=1)]
    return points_xyz

def create_colored_pointcloud(header, points_xyz, colors_bgr):
    # ... (기존과 동일)
    assert len(points_xyz) == len(colors_bgr)
    b, g, r = colors_bgr[:, 0], colors_bgr[:, 1], colors_bgr[:, 2]
    rgb_packed = np.array((r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | (b.astype(np.uint32)), dtype=np.uint32)
    rgb_packed_float = rgb_packed.view(np.float32)
    points_structured = np.zeros(len(points_xyz), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
    points_structured['x'] = points_xyz[:, 0]
    points_structured['y'] = points_xyz[:, 1]
    points_structured['z'] = points_xyz[:, 2]
    points_structured['rgb'] = rgb_packed_float
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1), PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1), PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
    return PointCloud2(header=header, height=1, width=len(points_structured), is_dense=False, is_bigendian=False, fields=fields, point_step=16, row_step=16 * len(points_structured), data=points_structured.tobytes())

# CuPy를 사용한 이미지 왜곡 보정 (Bilinear Interpolation)
def remap_cupy_manual(image_gpu, map1_gpu, map2_gpu):
    h, w, c = image_gpu.shape
    map1_gpu = cp.clip(map1_gpu, 0, w - 1)
    map2_gpu = cp.clip(map2_gpu, 0, h - 1)
    x0 = cp.floor(map1_gpu).astype(cp.int32)
    y0 = cp.floor(map2_gpu).astype(cp.int32)
    x1 = cp.clip(x0 + 1, 0, w - 1)
    y1 = cp.clip(y0 + 1, 0, h - 1)
    Ia = image_gpu[y0, x0]
    Ib = image_gpu[y1, x0]
    Ic = image_gpu[y0, x1]
    Id = image_gpu[y1, x1]
    wa = ((x1 - map1_gpu) * (y1 - map2_gpu))[..., None]
    wb = ((x1 - map1_gpu) * (map2_gpu - y0))[..., None]
    wc = ((map1_gpu - x0) * (y1 - map2_gpu))[..., None]
    wd = ((map1_gpu - x0) * (map2_gpu - y0))[..., None]
    return (wa * Ia + wb * Ib + wc * Ic + wd * Id).astype(image_gpu.dtype)

# CuPy를 사용한 COLORMAP_JET 적용
def apply_colormap_jet_cupy(normalized_values):
    # ... (기존과 동일)
    result = cp.zeros((normalized_values.shape[0], 3), dtype=cp.uint8)
    v = cp.array([0.0, 0.35, 0.65, 0.85, 1.0])
    c = cp.array([[128, 0, 0], [255, 0, 0], [0, 255, 255], [0, 0, 255], [0, 0, 128]], dtype=cp.uint8)
    for i in range(len(v) - 1):
        v0, v1 = v[i], v[i+1]; c0, c1 = c[i], c[i+1]
        mask = (normalized_values >= v0) & (normalized_values < v1)
        if cp.any(mask):
            interp = (normalized_values[mask] - v0) / (v1 - v0)
            color = c0 * (1 - interp[:, None]) + c1 * interp[:, None]
            result[mask] = color.astype(cp.uint8)
    mask_last = normalized_values >= v[-1]
    if cp.any(mask_last): result[mask_last] = c[-1]
    return result

# ==============================================================================
# 메인 통합 노드 클래스
# ==============================================================================

class LidarFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_fusion_node')

        # ... (파라미터 선언 및 가져오기는 기존과 동일) ...
        self.pipeline_started = False
        self.cam_info_loaded = False

        self.declare_parameter('serial_number', '000000000000')
        self.declare_parameter('camera_name', 'realsense_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('camera_info_path', '')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('lidar_frame', 'odom')
        self.declare_parameter('enable_yolo', False)
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('use_gpu', True)

        self.serial_number = self.get_parameter('serial_number').get_parameter_value().string_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value
        self.camera_info_path = self.get_parameter('camera_info_path').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        self.yolo_enabled = self.get_parameter('enable_yolo').get_parameter_value().bool_value
        self.yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value

        self.cupy_enabled = _CUPY_ENABLED and self.use_gpu
        if self.use_gpu and not _CUPY_ENABLED:
            self.get_logger().warn("GPU is enabled by parameter, but CuPy library is not found! Falling back to CPU.")

        self.yolo_model = None
        if self.yolo_enabled:
            if _YOLO_ENABLED: self.initialize_yolo()
            else: self.get_logger().error("YOLO is enabled, but 'ultralytics' is not found!"); self.yolo_enabled = False

        self.bridge = CvBridge()
        self.camera_matrix_cpu, self.dist_coeffs_cpu = None, None
        self.camera_matrix_cupy = None
        self.map1_gpu, self.map2_gpu = None, None
        self.lidar_to_cam_matrix = np.eye(4, dtype=np.float32)

        self.prev_time_for_fps = None
        self.smoothed_fps = 0.0

        self.qos_profile = QoSProfile(depth=10)
        # self.fused_image_pub = self.create_publisher(Image, f'/camera/{self.camera_name}/fused_image', self.qos_profile)
        self.raw_image_pub = self.create_publisher(Image, f'/camera/front_realsense_camera/color/image_raw', self.qos_profile)
        self.fused_image_pub = self.create_publisher(Image, f'/camera/front_realsense_camera/color/image_fused', self.qos_profile)
        self.fused_cloud_pub = self.create_publisher(PointCloud2, f'/camera/{self.camera_name}/fused_cloud', self.qos_profile)


        custom_lidar_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=20)
        self.utlidar_sub = self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.lidar_callback, custom_lidar_qos)
        # self.yrlidar_sub = self.create_subscription(PointCloud2, '/yrl_scan', self.lidar_callback, custom_lidar_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.frame_queue = queue.Queue(maxsize=1)
        self.lidar_buffer = deque(maxlen=30)

        self._load_camera_info()
        self._initialize_realsense()

        if self.pipeline_started:
            self.is_running = True
            self.frame_reader_thread = threading.Thread(target=self._frame_reader_loop)
            self.frame_reader_thread.daemon = True
            self.frame_reader_thread.start()
            timer_period = 1.0 / self.fps if self.fps > 0 else 0.033
            self.process_timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info(f"Lidar Fusion Node initialized. GPU Accelerated: {self.cupy_enabled}")

        # 마커 퍼블리셔 설정 (YOLO가 활성화된 경우에만)
        if self.yolo_enabled:
            self.setup_marker_publisher()

    def _load_camera_info(self):
        # ... (기존과 동일)
        try:
            with open(self.camera_info_path, 'r') as file:
                cam_params = yaml.safe_load(file)
                self.camera_matrix_cpu = np.array(cam_params['cameraMatrix']['data'], dtype=np.float32).reshape(3, 3)
                self.dist_coeffs_cpu = np.array(cam_params['distCoeffs']['data'], dtype=np.float32)
                self.cam_info_loaded = True

                if self.cupy_enabled:
                    self.camera_matrix_cupy = cp.asarray(self.camera_matrix_cpu)
                    map1_cpu, map2_cpu = cv2.initUndistortRectifyMap(
                        self.camera_matrix_cpu, self.dist_coeffs_cpu, None, self.camera_matrix_cpu,
                        (self.width, self.height), cv2.CV_32FC1)
                    self.map1_gpu = cp.asarray(map1_cpu)
                    self.map2_gpu = cp.asarray(map2_cpu)

                self.get_logger().info("Camera info loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load camera info: {e}")

    def _initialize_realsense(self):
        # ... (기존과 동일)
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_number != '000000000000': config.enable_device(self.serial_number)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, int(self.fps))
        try:
            self.pipeline.start(config)
            self.pipeline_started = True
            self.get_logger().info(f'RealSense pipeline started: {self.width}x{self.height} @ {self.fps}Hz')
        except Exception as e:
            self.get_logger().error(f'RealSense pipeline start failed: {e}'); raise e

    def initialize_yolo(self):
        # ... (기존과 동일)
        try:
            self.yolo_model = YOLO(self.yolo_model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize YOLO model: {e}"); self.yolo_enabled = False

    def lidar_callback(self, msg):
        # ... (기존과 동일)
        try:
            points_xyz = pointcloud2_to_xyz_array(msg)
            if points_xyz.shape[0] > 0: self.lidar_buffer.append((RclpyTime.from_msg(msg.header.stamp), points_xyz))
        except ValueError as e:
            self.get_logger().warn(f"Could not parse PointCloud2: {e}", throttle_duration_sec=5.0)

    def _frame_reader_loop(self):
        # ... (기존과 동일)
        while self.is_running and rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                if not frames: continue
                color_frame = frames.get_color_frame()
                if color_frame:
                    # 원본 이미지 발행
                    raw_img_data = np.asanyarray(color_frame.get_data())
                    raw_msg = self.bridge.cv2_to_imgmsg(raw_img_data, encoding="bgr8")
                    raw_msg.header.stamp = self.get_clock().now().to_msg() # 근사 시간
                    raw_msg.header.frame_id = self.camera_frame
                    self.raw_image_pub.publish(raw_msg)

                    # 큐에 데이터 넣기
                    try: self.frame_queue.get_nowait()
                    except queue.Empty: pass
                    self.frame_queue.put(raw_img_data)
            except RuntimeError as e:
                self.get_logger().warn(f"Frame acquisition error: {e}", throttle_duration_sec=5.0)

    def setup_marker_publisher(self):
        """
        마커 발행에 필요한 파라미터와 퍼블리셔를 초기화합니다.
        (기존 노드의 __init__ 마지막 부분에서 호출하세요.)
        """
        self.get_logger().info('Setting up YOLO marker publisher...')

        # --- 마커 관련 파라미터 선언 ---
        # (기존 노드의 파라미터와 이름이 겹치지 않도록 'marker_' 접두사 사용)
        self.declare_parameter('marker_target_class_id', 'person') # 이 파라미터는 현재 사용되지 않음 (클래스 0으로 하드코딩)
        self.declare_parameter('marker_scale', 0.4)
        self.declare_parameter('marker_lifetime', 0.5)

        # --- 파라미터 값 가져오기 ---
        self.marker_target_class_id_ = self.get_parameter('marker_target_class_id').get_parameter_value().string_value
        self.marker_scale_ = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.marker_lifetime_sec_ = self.get_parameter('marker_lifetime').get_parameter_value().double_value

        self.get_logger().info(f"Marker filtering for class ID: '{self.marker_target_class_id_}' (Note: Hardcoded to class 0 in process_frame)")

        # --- 마커 발행 퍼블리셔 ---
        self.marker_publisher_ = self.create_publisher(
            MarkerArray,
            '/yolo_markers',  # RViz에서 구독할 토픽
            10)

        # 마지막에 발행한 마커 수를 추적 (오래된 마커 삭제용)
        self.last_marker_count_ = 0

    def publish_simple_markers(self, object_positions: list, header: Header):
        """
        Calculated object_positions list [(pos, count), ...]를 받아
        RViz 마커로 직접 변환 후 발행합니다.
        (기존 publish_yolo_markers 함수를 대체합니다)
        """
        marker_array = MarkerArray()
        current_marker_id = 0

        # object_positions는 (pos, count) 튜플의 리스트입니다.
        for (pos, count) in object_positions:
            # --- 대상 객체에 대한 마커 생성 ---
            marker = Marker()
            marker.header = header # process_frame에서 받은 헤더 사용
            marker.ns = "person_detections" # 네임스페이스는 동일하게 유지
            marker.id = current_marker_id
            current_marker_id += 1

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # [핵심] 'pos' (x,y,z)를 직접 pose.position에 할당
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0 # 기본 방향

            # 기존 마커 파라미터 사용
            marker.scale.x = self.marker_scale_
            marker.scale.y = self.marker_scale_
            marker.scale.z = self.marker_scale_

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker.lifetime = Duration(seconds=self.marker_lifetime_sec_).to_msg()

            marker_array.markers.append(marker)

        # --- 이전에 발행된 마커 중 불필요한 마커 삭제 ---
        # (이 로직은 self.last_marker_count_를 사용하므로 동일하게 유지)
        if self.last_marker_count_ > current_marker_id:
            for i in range(current_marker_id, self.last_marker_count_):
                delete_marker = Marker()
                delete_marker.header = header
                delete_marker.ns = "person_detections"
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

        self.last_marker_count_ = current_marker_id

        # 생성된 마커 배열 발행
        self.marker_publisher_.publish(marker_array)

    def calculate_object_positions(self, yolo_boxes_xyxy, pixel_x, pixel_y, points_camera_frame, is_gpu_array=False):
        """
        YOLO 바운딩 박스 내의 LiDAR 포인트들의 평균 카메라 좌표계 위치를 계산합니다.
        [수정] 배경/아웃라이어 포인트 제거를 위해 Z-좌표의 중앙값(Median)을 기준으로
               +/- 30% 범위 내의 포인트들만 필터링하여 평균을 계산합니다.
        """
        lib = cp if is_gpu_array else np
        object_positions = []

        if yolo_boxes_xyxy is None or yolo_boxes_xyxy.shape[0] == 0 or pixel_x.shape[0] == 0:
            return object_positions

        # YOLO 박스가 GPU에 있으면 CPU로 복사
        if hasattr(yolo_boxes_xyxy, 'cpu'): # PyTorch 텐서인지 확인
            boxes_cpu = yolo_boxes_xyxy.cpu().numpy()
        else: # 이미 NumPy 배열이라고 가정
            boxes_cpu = yolo_boxes_xyxy

        for box in boxes_cpu:
            x1, y1, x2, y2 = box

            mask_x = (pixel_x >= x1) & (pixel_x <= x2)
            mask_y = (pixel_y >= y1) & (pixel_y <= y2)
            inside_box_mask = mask_x & mask_y

            points_in_box = points_camera_frame[inside_box_mask]
            num_points_in_box = points_in_box.shape[0]

            if num_points_in_box == 0:
                continue

            elif num_points_in_box <= 5:
                # 포인트가 너무 적으면 통계적 필터링이 무의미하므로 단순 평균 사용
                average_position = lib.mean(points_in_box, axis=0)
                num_filtered_points = num_points_in_box

            else:
                # [수정] Z-좌표의 중앙값(Median)을 사용하여 아웃라이어(배경) 필터링
                z_points = points_in_box[:, 2]

                # 중앙값 계산 (아웃라이어에 강건함)
                median_z = lib.median(z_points)

                # 중앙값의 30%를 임계값(threshold)으로 설정 (너무 작지 않게 최소값 보장)
                z_threshold = lib.maximum(0.3 * median_z, 0.1) # 최소 10cm

                # Z좌표가 (중앙값 - 임계값) ~ (중앙값 + 임계값) 범위 내에 있는 포인트만 선택
                filter_mask = (lib.abs(z_points - median_z) < z_threshold)

                filtered_points = points_in_box[filter_mask]
                num_filtered_points = filtered_points.shape[0]

                if num_filtered_points > 0:
                    # 필터링된 포인트들의 평균 위치 계산
                    average_position = lib.mean(filtered_points, axis=0)
                else:
                    # 필터링 후 남은 포인트가 없으면 이 객체는 무시
                    continue

            # GPU 결과인 경우 CPU로 복사
            if is_gpu_array:
                average_position = cp.asnumpy(average_position)

            # [요청 사항] X 좌표를 0으로 고정하여 YZ 평면에 투영
            average_position[1] = 0.0

            object_positions.append((average_position, num_filtered_points))

        return object_positions

    def process_frame(self):
        now = self.get_clock().now()
        if self.prev_time_for_fps is not None:
            dt = (now - self.prev_time_for_fps).nanoseconds / 1e9
            if dt > 0: self.smoothed_fps = 0.95 * self.smoothed_fps + 0.05 * (1.0 / dt)
        self.prev_time_for_fps = now

        try:
            frame_cpu = self.frame_queue.get_nowait()
        except queue.Empty:
            return

        try:
            trans = self.tf_buffer.lookup_transform(self.camera_frame, self.lidar_frame, RclpyTime(), timeout=Duration(seconds=0.05))
            t = trans.transform.translation; q = trans.transform.rotation
            self.lidar_to_cam_matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self.lidar_to_cam_matrix[:3, 3] = [t.x, t.y, t.z]
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=1.0)
            if self.show_preview: cv2.imshow(f"Fused Stream", frame_cpu); cv2.waitKey(1)
            return

        yolo_results = None
        object_positions = [] # [신규] 두 경로에서 공통으로 사용할 변수 초기화

        if self.cupy_enabled:
            # --- GPU 경로 ---
            try:
                # 1. CPU -> GPU 이미지 전송 및 왜곡 보정
                frame_gpu = cp.asarray(frame_cpu)
                undistorted_gpu = remap_cupy_manual(frame_gpu, self.map1_gpu, self.map2_gpu)

                # 2. YOLO 처리 (먼저 실행)
                if self.yolo_enabled:
                    processed_image_for_yolo = cp.asnumpy(undistorted_gpu)
                    # [수정] classes=0 (person)만 감지, conf=0.5 이상
                    yolo_results = self.yolo_model(processed_image_for_yolo, classes=0, conf=0.5, verbose=False)
                    processed_image = yolo_results[0].plot()
                else:
                    processed_image = cp.asnumpy(undistorted_gpu)

                # 3. 라이다 융합
                if self.lidar_buffer:
                    points_lidar_frame = np.vstack([points for _, points in self.lidar_buffer])
                    if points_lidar_frame.shape[0] > 0:
                        points_lidar_gpu = cp.asarray(points_lidar_frame)
                        transform_gpu = cp.asarray(self.lidar_to_cam_matrix)

                        points_lidar_h_gpu = cp.hstack((points_lidar_gpu, cp.ones((points_lidar_gpu.shape[0], 1))))
                        points_cam_h_gpu = (transform_gpu @ points_lidar_h_gpu.T).T

                        in_front_mask_gpu = points_cam_h_gpu[:, 2] > 0.1
                        points_cam_frame_gpu = points_cam_h_gpu[in_front_mask_gpu, :3]
                        points_lidar_in_front_gpu = points_lidar_gpu[in_front_mask_gpu]

                        if points_cam_frame_gpu.shape[0] > 0:
                            projected_gpu = (self.camera_matrix_cupy @ points_cam_frame_gpu.T).T
                            Z_gpu = projected_gpu[:, 2]
                            valid_z_mask_gpu = Z_gpu > 1e-6

                            points_cam_frame_valid_z_gpu = points_cam_frame_gpu[valid_z_mask_gpu]
                            points_lidar_valid_z_gpu = points_lidar_in_front_gpu[valid_z_mask_gpu]

                            px_gpu = projected_gpu[valid_z_mask_gpu, 0] / Z_gpu[valid_z_mask_gpu]
                            py_gpu = projected_gpu[valid_z_mask_gpu, 1] / Z_gpu[valid_z_mask_gpu]
                            z_values_gpu = points_cam_frame_valid_z_gpu[:, 2]

                            bounds_mask_gpu = (px_gpu >= 0) & (px_gpu < self.width) & (py_gpu >= 0) & (py_gpu < self.height)

                            final_points_cam_gpu = points_cam_frame_valid_z_gpu[bounds_mask_gpu]
                            final_points_lidar_gpu = points_lidar_valid_z_gpu[bounds_mask_gpu]

                            valid_px_gpu = px_gpu[bounds_mask_gpu] # [수정] int 캐스팅 제거 (계산 함수에서 처리)
                            valid_py_gpu = py_gpu[bounds_mask_gpu]
                            valid_z_gpu = z_values_gpu[bounds_mask_gpu]

                            if valid_px_gpu.shape[0] > 0:

                                # [수정] 동적 정규화
                                min_z = cp.min(valid_z_gpu)
                                max_z = cp.max(valid_z_gpu)
                                range_z = max_z - min_z

                                if range_z > 1e-5: # 0으로 나누기 방지
                                    norm_z_gpu = (valid_z_gpu - min_z) / range_z
                                else:
                                    norm_z_gpu = cp.full_like(valid_z_gpu, 0.5) # 모두 같은 깊이면 중간색

                                norm_z_gpu = cp.clip(norm_z_gpu, 0.0, 1.0)
                                # [기존] norm_z_gpu = cp.clip(valid_z_gpu / 3.0, 0.0, 1.0) # 3m 기준으로 정규화

                                colors_gpu = apply_colormap_jet_cupy(norm_z_gpu)

                                # CPU로 복사 (이미지 표기 및 PCL 생성용)
                                valid_px_cpu = cp.asnumpy(valid_px_gpu).astype(np.int32)
                                valid_py_cpu = cp.asnumpy(valid_py_gpu).astype(np.int32)
                                colors_cpu = cp.asnumpy(colors_gpu)
                                final_points_lidar_cpu = cp.asnumpy(final_points_lidar_gpu)

                                for i in range(len(valid_px_cpu)):
                                    cv2.circle(processed_image, (valid_px_cpu[i], valid_py_cpu[i]), 4, tuple(colors_cpu[i].tolist()), -1)

                                cloud_header = Header(stamp=now.to_msg(), frame_id=self.lidar_frame)
                                fused_cloud_msg = create_colored_pointcloud(cloud_header, final_points_lidar_cpu, colors_cpu)
                                self.fused_cloud_pub.publish(fused_cloud_msg)

                                # 4. [수정] YOLO 객체 위치 계산
                                if self.yolo_enabled and yolo_results:
                                    object_positions = self.calculate_object_positions(
                                        yolo_results[0].boxes.xyxy,
                                        valid_px_gpu, # int 캐스팅 전의 float 값 전달
                                        valid_py_gpu,
                                        final_points_cam_gpu,
                                        is_gpu_array=True
                                    )
                                    # (로깅은 메시지 발행 후로 이동)

            except Exception as e:
                self.get_logger().error(f"GPU processing failed: {e}\n{traceback.format_exc()}", throttle_duration_sec=2.0)
                processed_image = frame_cpu

        else:
            # --- CPU 경로 ---
            undistorted_image = cv2.undistort(frame_cpu, self.camera_matrix_cpu, self.dist_coeffs_cpu)

            if self.yolo_enabled:
                # [수정] classes=0 (person)만 감지, conf=0.5 이상
                yolo_results = self.yolo_model(undistorted_image, classes=0, conf=0.5, verbose=False)
                processed_image = yolo_results[0].plot()
            else:
                processed_image = undistorted_image

            if self.lidar_buffer:
                points_lidar_frame = np.vstack([points for _, points in self.lidar_buffer])
                if points_lidar_frame.shape[0] > 0:
                    points_lidar_h = np.hstack((points_lidar_frame, np.ones((points_lidar_frame.shape[0], 1))))
                    points_cam_h = (self.lidar_to_cam_matrix @ points_lidar_h.T).T

                    in_front_mask = points_cam_h[:, 2] > 0.1
                    points_cam_frame = points_cam_h[in_front_mask, :3]
                    points_lidar_in_front = points_lidar_frame[in_front_mask]

                    if points_cam_frame.shape[0] > 0:
                        projected = (self.camera_matrix_cpu @ points_cam_frame.T).T
                        Z = projected[:, 2]
                        valid_z_mask = Z > 1e-6

                        points_cam_frame_valid_z = points_cam_frame[valid_z_mask]
                        points_lidar_valid_z = points_lidar_in_front[valid_z_mask]

                        px_float = projected[valid_z_mask, 0] / Z[valid_z_mask]
                        py_float = projected[valid_z_mask, 1] / Z[valid_z_mask]
                        z_values = points_cam_frame_valid_z[:, 2]

                        bounds_mask = (px_float >= 0) & (px_float < self.width) & (py_float >= 0) & (py_float < self.height)

                        final_points_cam = points_cam_frame_valid_z[bounds_mask]
                        final_points_lidar = points_lidar_valid_z[bounds_mask]

                        valid_px_float = px_float[bounds_mask]
                        valid_py_float = py_float[bounds_mask]
                        valid_z = z_values[bounds_mask]

                        if valid_px_float.shape[0] > 0:

                            # [수정] 동적 정규화
                            min_z = np.min(valid_z)
                            max_z = np.max(valid_z)
                            range_z = max_z - min_z

                            if range_z > 1e-5: # 0으로 나누기 방지
                                norm_z = (valid_z - min_z) / range_z
                            else:
                                norm_z = np.full_like(valid_z, 0.5) # 모두 같은 깊이면 중간값

                            norm_z = np.clip(norm_z, 0, 1)
                            # [기존] norm_z = np.clip(valid_z / 3.0, 0, 1)

                            norm_z_scaled = (norm_z * 255).astype(np.uint8)
                            colors = cv2.applyColorMap(norm_z_scaled, cv2.COLORMAP_JET).reshape(-1, 3)

                            valid_px_int = valid_px_float.astype(int)
                            valid_py_int = valid_py_float.astype(int)

                            for i in range(len(valid_px_int)):
                                cv2.circle(processed_image, (valid_px_int[i], valid_py_int[i]), 4, tuple(colors[i].tolist()), -1)

                            cloud_header = Header(stamp=now.to_msg(), frame_id=self.lidar_frame)
                            fused_cloud_msg = create_colored_pointcloud(cloud_header, final_points_lidar, colors)
                            self.fused_cloud_pub.publish(fused_cloud_msg)

                            # 4. [수정] YOLO 객체 위치 계산
                            if self.yolo_enabled and yolo_results:
                                object_positions = self.calculate_object_positions(
                                    yolo_results[0].boxes.xyxy,
                                    valid_px_float, # int 캐스팅 전의 float 값 전달
                                    valid_py_float,
                                    final_points_cam,
                                    is_gpu_array=False
                                )
                                # (로깅은 메시지 발행 후로 이동)

        # ======================================================================
        # 6. [신규] YOLO 마커 발행
        # ======================================================================
        if self.yolo_enabled and object_positions:
            # 헤더 생성: 타임스탬프는 현재 시간, 프레임 ID는 카메라 프레임
            marker_header = Header(stamp=now.to_msg(), frame_id=self.camera_frame)
            self.publish_simple_markers(object_positions, marker_header)

            # (디버깅 로그)
            # self.get_logger().info(f"Published {len(object_positions)} markers.")

        # ======================================================================
        # 7. 최종 이미지 발행 (기존 로직)
        # ======================================================================
        fps_text = f"FPS: {self.smoothed_fps:.2f}"
        cv2.putText(processed_image, fps_text, (self.width - 220, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        fused_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
        fused_msg.header.stamp = now.to_msg()
        fused_msg.header.frame_id = self.camera_frame
        self.fused_image_pub.publish(fused_msg)

        if self.show_preview:
            cv2.imshow(f"Fused Stream: {self.camera_name}", processed_image)
            cv2.waitKey(1)

    def destroy_node(self):
        # ... (기존과 동일) ...
        self.is_running = False
        if hasattr(self, 'frame_reader_thread') and self.frame_reader_thread.is_alive(): self.frame_reader_thread.join()
        if self.pipeline_started: self.pipeline.stop()
        if self.show_preview: cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarFusionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        print(f"Node initialization failed: {e}\n{traceback.format_exc()}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()