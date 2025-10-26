import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
import numpy as np
import math
import tf2_ros
from scipy.spatial.transform import Rotation
import sys
import traceback # 상세 오류 로깅을 위해 임포트
from numpy.lib.stride_tricks import as_strided 

# ==============================================================================
# GPU 라이브러리 임포트 (CuPy는 필수입니다)
# ==============================================================================
try:
    import cupy as cp
    _GPU_ENABLED = True
except ImportError as e:
    _GPU_ENABLED = False
    print("="*50)
    print(f"WARNING: CuPy 라이브러리를 찾을 수 없습니다. (Error: {e})")
    print("이 노드는 CuPy가 필수입니다. 설치 후 다시 실행해주세요.")
    print("="*50)
    sys.exit(1)

# ==============================================================================
# 커스텀 CUDA 커널 (Atomic Min 연산을 위해 사용)
# ==============================================================================
# PointCloud2를 LaserScan으로 변환할 때, 같은 각도(bin)에 여러 포인트가 투영될 경우
# 가장 가까운 거리(min range)를 선택하기 위해 원자적 최소값(Atomic Min) 연산이 필요합니다.
float_atomic_min_kernel = cp.RawKernel(r'''
extern "C" __global__
void float_atomic_min(float* output, const int* indices, const float* values, int N)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i < N) {
        float* address = &output[indices[i]];
        float val = values[i];
        
        unsigned int* address_as_int = (unsigned int*)address;
        unsigned int old_int = *address_as_int;
        float old_float = __int_as_float(old_int);

        while (val < old_float) {
            unsigned int assumed_int = __float_as_int(old_float);
            unsigned int val_int = __float_as_int(val);
            
            unsigned int result_int = atomicCAS(address_as_int, assumed_int, val_int);
            
            if (result_int == assumed_int) {
                break;
            }
            
            old_float = __int_as_float(result_int);
        }
    }
}
''', 'float_atomic_min')
# ==============================================================================
# PointCloud2 메시지 처리를 위한 헬퍼 함수 (CPU 파싱 최적화)
# ==============================================================================
def pointcloud2_to_xyz_array(cloud_msg: PointCloud2, remove_nans=True):
    """
    [수정됨] sensor_msgs/PointCloud2 메시지를 (N, 3) 형태의 NumPy 배열로 변환합니다.
    np.ndarray 생성자를 사용하여 원본 'data' 버퍼에 대한 뷰(View)를 생성합니다.
    (메모리 복사 없음)
    """
    fields_offsets = {field.name: field.offset for field in cloud_msg.fields}
    
    # 필수 필드 확인
    if not all(f in fields_offsets for f in ['x', 'y', 'z']):
        raise ValueError("PointCloud2 message must have 'x', 'y', and 'z' fields.")

    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0:
        return np.array([], dtype=np.float32).reshape(0, 3)
        
    x_offset = fields_offsets['x']
    y_offset = fields_offsets['y']
    z_offset = fields_offsets['z']
    point_step = cloud_msg.point_step
    
    # data_buffer는 NumPy 배열이 아닌 원시 bytes 객체여야 합니다.
    # (np.frombuffer는 복사본을 만들 수 있으므로 msg.data를 직접 사용)
    data_buffer = cloud_msg.data

    # 1. np.ndarray 생성자를 사용하여 각 필드에 대한 뷰(View)를 생성합니다.
    #    이 방식은 메모리 복사를 발생시키지 않습니다.
    
    # x 좌표 뷰
    x_view = np.ndarray(
        shape=(num_points,),
        dtype=np.float32,
        buffer=data_buffer,
        offset=x_offset,
        strides=(point_step,)
    )
    # y 좌표 뷰
    y_view = np.ndarray(
        shape=(num_points,),
        dtype=np.float32,
        buffer=data_buffer,
        offset=y_offset,
        strides=(point_step,)
    )
    # z 좌표 뷰
    z_view = np.ndarray(
        shape=(num_points,),
        dtype=np.float32,
        buffer=data_buffer,
        offset=z_offset,
        strides=(point_step,)
    )

    # 2. 뷰들을 쌓아 최종 배열을 생성합니다. (이 시점에서 1회 복사 발생)
    #    .copy()를 호출하여 읽기 전용(read-only) 플래그 문제를 방지하고
    #    후속 NumPy/CuPy 연산(예: NaN 제거)을 안전하게 만듭니다.
    points = np.stack([x_view, y_view, z_view], axis=1).copy()

    if remove_nans:
        # NaN 값 제거
        points = points[~np.isnan(points).any(axis=1)]

    return points


# ==============================================================================
# 메인 노드 클래스 (GPU 가속화 적용)
# ==============================================================================

class CuPyPointCloudToLaserScan(Node):
    """
    PointCloud2를 LaserScan으로 변환하는 작업을 CuPy를 사용해 GPU에서
    고속으로 처리하는 노드.
    """
    def __init__(self):
        super().__init__('cupy_pointcloud_to_laserscan')

        if not _GPU_ENABLED:
            self.get_logger().error("CuPy가 임포트되지 않아 노드를 종료합니다.")
            return

        # [파라미터 선언 및 가져오기 생략 - 이전 코드와 동일]
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_height', -0.05)
        self.declare_parameter('max_height', 0.05)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 360.0) # 0.5도 단위
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('use_inf', True)
        self.declare_parameter('lidar_frame', 'odom') # 예시. 실제 프레임 ID로 변경

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.use_inf = self.get_parameter('use_inf').get_parameter_value().bool_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value

        self.num_bins = int(round((self.angle_max - self.angle_min) / self.angle_increment))
        if self.num_bins <= 0:
            self.get_logger().error("파라미터 오류: LaserScan의 빈 개수가 0 이하입니다. (num_bins)")
            sys.exit(1)

        # TF 리스너 및 버퍼
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 퍼블리셔 및 서브스크라이버
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        custom_lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'cloud_in',
            self.cloud_callback,
            qos_profile=custom_lidar_qos
        )

        # GPU 메모리에 캐시될 TF 변환 행렬을 위한 멤버 변수 및 스트림
        self.tf_matrix_gpu = None # (CuPy 배열)
        self.msg_frame = None
        self.stream = cp.cuda.Stream() # CuPy 연산 및 비동기 전송을 위한 스트림
        
        # GPU 메모리 사전 할당 (매번 생성/삭제 방지)
        # scan_ranges_gpu는 항상 재사용됩니다.
        self.scan_ranges_gpu = cp.full(self.num_bins, cp.inf, dtype=cp.float32)
        
        # TF 업데이트 타이머
        self.tf_timer = self.create_timer(0.05, self.update_transform)

        self.get_logger().info(f"CuPy 가속화 PointCloud->LaserScan 노드 시작. (bins: {self.num_bins})")
        
    def update_transform(self):
        """
        [20Hz Timer] 주기적으로 TF를 조회하여 LiDAR -> Target 프레임 변환 행렬을
        GPU 메모리(self.tf_matrix_gpu)에 직접 캐시합니다.
        """
        try:
            
            if self.msg_frame is not None: 
                self.lidar_frame = self.msg_frame
                # self.get_logger().info(f"Updating TF transform...{self.lidar_frame}")
            # 1. [CPU] TF 조회
            trans_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,      # Target Frame
                self.lidar_frame,         # Source Frame (메시지에서 직접 가져옴)
                rclpy.time.Time(),       # 메시지 시간 기준으로 조회 (더 정확함)
                timeout=Duration(seconds=0.1) 
            )

            # 2. [CPU] TF 변환 행렬 생성
            t = trans_stamped.transform.translation
            q = trans_stamped.transform.rotation
            rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            tf_matrix_cpu = np.eye(4, dtype=np.float32)
            tf_matrix_cpu[:3, :3] = rotation_matrix
            tf_matrix_cpu[:3, 3] = [t.x, t.y, t.z]
            
            # 3. [CPU -> GPU] 변환 행렬 업로드 (작은 행렬이므로 오버헤드 미미)
            self.tf_matrix_gpu = cp.asarray(tf_matrix_cpu)
            # self.get_logger().info(f'TF 변환 success {self.lidar_frame} -> {self.target_frame}')

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF 변환 실패 {self.lidar_frame} -> {self.target_frame}: {ex}', throttle_duration_sec=2.0)
            return

    def cloud_callback(self, msg: PointCloud2):
        
        self.msg_frame = msg.header.frame_id
        # 1. [Guard Clause] TF 행렬 준비 확인
        if self.tf_matrix_gpu is None:
            self.get_logger().warn("TF 변환 행렬이 유효하지 않습니다.", throttle_duration_sec=2.0)
            return
        
        try:
            # 2. [CPU] PointCloud2 파싱 (최적화된 함수 사용)
            points_cpu = pointcloud2_to_xyz_array(msg)
            if points_cpu.shape[0] == 0:
                self.publish_empty_scan(msg.header.stamp); return

            # 3. [CPU -> GPU] 데이터 비동기 업로드 및 GPU 연산 시작
            # with self.stream: 블록 내 모든 연산은 비동기적으로 스케줄링됩니다.
            
            with self.stream:
                points_gpu = cp.asarray(points_cpu, dtype=cp.float32)
                
                # 4. [GPU] 좌표 변환 (단일 행렬 곱셈)
                points_homogeneous = cp.hstack((points_gpu, cp.ones((points_gpu.shape[0], 1), dtype=cp.float32)))
                transformed_points = cp.dot(points_homogeneous, self.tf_matrix_gpu.T)
                points_gpu = transformed_points[:, :3]
                
                # 5. [GPU] 높이 필터링
                z = points_gpu[:, 2]
                height_mask = (z >= self.min_height) & (z <= self.max_height)
                points_filtered = points_gpu[height_mask]
                if points_filtered.shape[0] == 0:
                    self.publish_empty_scan(msg.header.stamp); return
                

                # 6. [GPU] 2D 투영 및 극좌표계 변환
                x = points_filtered[:, 0]; y = points_filtered[:, 1]
                ranges = cp.sqrt(x**2 + y**2)
                angles = cp.arctan2(y, x)
                
                # 7. [GPU] 거리 및 각도 필터링
                range_mask = (ranges >= self.range_min) & (ranges <= self.range_max)
                valid_ranges = ranges[range_mask]
                valid_angles = angles[range_mask]
                # self.get_logger().info(f'pointcloud input received. processing...{points_filtered.shape[0]} points')
                self.get_logger().info(f'pointcloud input received. processing...{valid_ranges.shape[0]} points')
                if valid_ranges.shape[0] == 0:
                    self.publish_empty_scan(msg.header.stamp); return
                
                # 8. [GPU] 각도를 스캔 빈(bin) 인덱스로 변환
                indices = ((valid_angles - self.angle_min) / self.angle_increment).astype(cp.int32)
                
                # 9. [GPU] 유효한 인덱스만 필터링
                bin_mask = (indices >= 0) & (indices < self.num_bins)
                valid_ranges = valid_ranges[bin_mask]
                valid_indices = indices[bin_mask]
                
                num_valid_points = valid_indices.shape[0]
                if num_valid_points == 0:
                    self.publish_empty_scan(msg.header.stamp); return

                # 10. [GPU] 스캔 빈 채우기 (커스텀 Atomic Min 커널 사용)
                self.scan_ranges_gpu.fill(cp.inf)
                
                block_size = 256
                grid_size = (num_valid_points + block_size - 1) // block_size
                
                float_atomic_min_kernel(
                    (grid_size,),                   
                    (block_size,),                  
                    (self.scan_ranges_gpu,          
                     valid_indices,                 
                     valid_ranges,                  
                     num_valid_points),             
                    stream=self.stream              # 비동기 실행
                )
                
                # 11. [GPU] 'inf' 값 처리
                if not self.use_inf:
                    self.scan_ranges_gpu[cp.isinf(self.scan_ranges_gpu)] = self.range_max + 1.0

                # 12. [GPU -> CPU] 최종 스캔 데이터 다운로드
                # .get()은 이 시점에서 스트림 동기화를 암묵적으로 수행합니다.
                scan_ranges_cpu = self.scan_ranges_gpu.get()
                

            # 13. [CPU] LaserScan 메시지 생성 및 발행 (NumPy -> List 변환 오버헤드 발생)
            scan_msg = LaserScan()
            scan_msg.header.stamp = msg.header.stamp
            scan_msg.header.frame_id = self.target_frame
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.0
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            # 여기서 .tolist() 변환이 CPU 부하를 유발하지만, ROS 메시지 발행에 필수적입니다.
            scan_msg.ranges = scan_ranges_cpu.tolist()
            # self.get_logger().info("PointCloud를 LaserScan으로 성공적으로 변환 및 발행.")
            
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            # 상세 traceback 로깅 추가 (디버깅 효율 증대)
            error_trace = traceback.format_exc()
            self.get_logger().error(f"--- 상세 PointCloud 처리 TRACEBACK ---")
            for line in error_trace.splitlines():
                self.get_logger().error(f" | {line}")
            self.get_logger().error(f"----------------------------------------")
            self.get_logger().error(f"PointCloud 처리 중 오류: {e}", throttle_duration_sec=1.0)
            

    def publish_empty_scan(self, stamp):
        """유효한 포인트가 없을 때 빈 LaserScan 메시지를 발행합니다."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        val = cp.inf if self.use_inf else self.range_max + 1.0
        scan_msg.ranges = [float(val)] * self.num_bins
        
        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    if not _GPU_ENABLED:
        print("CuPy가 없어 노드를 실행할 수 없습니다.")
        return
        
    node = CuPyPointCloudToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
