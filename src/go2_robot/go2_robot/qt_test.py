import sys
import os
import signal
import json
import rclpy
import torch
import logging 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# 🚨 [수정] QTableWidget 사용을 위한 클래스 모두 임포트
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QDoubleSpinBox, QPushButton, QCheckBox, QTextBrowser, QProgressBar, QTableWidget, QTableWidgetItem, QHeaderView
from PyQt5.QtCore import QTimer, pyqtSignal, Qt, QObject 
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
import numpy as np

# 🚨 [필수 임포트] ROS 메시지 및 API ID 정의
from unitree_api.msg import Request 
from rcl_interfaces.msg import Log 
from unitree_go.msg import LowState, BmsState 

# --- API ID 정의 ---
SPORT_API_ID_DAMP = 1001
SPORT_API_ID_BALANCESTAND = 1002
SPORT_API_ID_STANDUP = 1004
SPORT_API_ID_MOVE = 1008
AVOID_API_ID_SWITCH_SET = 1001 
AVOID_API_ID_MOVE = 1003
AVOID_TOPIC = '/api/obstacles_avoid/request'

# ----------------------------------------------------
# 1. ROS 2 발행/구독 노드 (LowState 구독 및 데이터 전송)
# ----------------------------------------------------
class RobotControlNode(Node, QObject):
    new_frame_signal = pyqtSignal(QPixmap) 
    new_log_signal = pyqtSignal(str) 
    # 🚨 [수정] 상태 업데이트 시그널: SOC, MaxTemp, 12개 온도 리스트(list)를 전송
    state_update_signal = pyqtSignal(int, int, list) 

    def __init__(self, topic_name, label_widget: QLabel):
        Node.__init__(self, f'robot_gui_control_node') 
        QObject.__init__(self)
        
        self.bridge = CvBridge()
        self.label_size = label_widget.size()
        self.subscription = self.create_subscription(Image, topic_name, self.listener_callback, 10)
        
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.avoid_pub = self.create_publisher(Request, AVOID_TOPIC, 10)

        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)
        
        # 🚨 [수정] 토픽 이름: /lowstate
        self.lowstate_sub = self.create_subscription(LowState, '/lowstate', self.lowstate_callback, 10)

    def lowstate_callback(self, msg: LowState):
        """LowState 메시지에서 배터리 및 모터 온도 추출 (12개 온도 리스트 추출)"""
        soc = msg.bms_state.soc 
        max_temp = 0
        temps_12 = []
        for i in range(12): 
            temp = msg.motor_state[i].temperature
            max_temp = max(max_temp, temp)
            temps_12.append(temp)
            
        self.state_update_signal.emit(soc, max_temp, temps_12) # 🚨 12개 온도 리스트 전송

    # --- (나머지 제어 및 이미지 처리 함수들은 이전과 동일) ---
    def log_callback(self, msg: Log):
        level_map = {20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = level_map.get(msg.level, f"LVL_{msg.level}")
        log_message = f"[{level_str}] [{msg.name}]: {msg.msg}"
        self.new_log_signal.emit(log_message)

    def send_sport_command(self, api_id: int, vx: float=0.0, vy: float=0.0, vyaw: float=0.0):
        req_msg = Request()
        req_msg.header.identity.api_id = api_id
        if api_id == SPORT_API_ID_MOVE:
            p = {"x": vx, "y": vy, "z": vyaw}
            req_msg.parameter = json.dumps(p)
        else:
            req_msg.parameter = ""
        self.sport_pub.publish(req_msg)
        self.get_logger().info(f"Sent Sport Command ID {api_id}")

    def send_avoidance_move(self, vx: float, vy: float, vyaw: float):
        req_msg = Request()
        req_msg.header.identity.api_id = AVOID_API_ID_MOVE
        p = {"x": vx, "y": vy, "z": vyaw} 
        req_msg.parameter = json.dumps(p)
        self.avoid_pub.publish(req_msg)
        self.get_logger().info(f"Sent AVOID MOVE: vx={vx}, vy={vy}, vyaw={vyaw}")

    def set_avoidance_switch(self, enable: bool):
        req_msg = Request()
        req_msg.header.identity.api_id = AVOID_API_ID_SWITCH_SET
        p = {"data": enable}
        req_msg.parameter = json.dumps(p)
        self.avoid_pub.publish(req_msg)
        self.get_logger().info(f"Sent Avoidance SWITCH_SET: {'Enabled' if enable else 'Disabled'}")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image).scaled(self.label_size.width(), self.label_size.height(), Qt.KeepAspectRatio)
            self.new_frame_signal.emit(pixmap)
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')


# ----------------------------------------------------
# 2. 메인 Qt 애플리케이션 클래스 (QTableWidget 초기화 및 업데이트)
# ----------------------------------------------------
try:
    package_share_dir = get_package_share_directory('go2_robot')
    UI_FILE_PATH = os.path.join(package_share_dir, 'untitled.ui')
    FormClass, BaseClass = uic.loadUiType(UI_FILE_PATH)
except Exception as e:
    print(f"Error loading UI file: {e}. Using generic QMainWindow.")
    FormClass, BaseClass = QMainWindow, QMainWindow 

class MainWindow(BaseClass, FormClass):
    def __init__(self): 
        super().__init__()
        
        if hasattr(self, 'setupUi'): 
             self.setupUi(self) 
             
        # 🌟 [QTableWidget 초기화] 🌟
        if hasattr(self, 'motor_temp_table'):
            self.motor_temp_table.setRowCount(3)
            self.motor_temp_table.setColumnCount(4)
            self.motor_temp_table.setHorizontalHeaderLabels(['FL', 'FR', 'RL', 'RR'])
            self.motor_temp_table.setVerticalHeaderLabels(['Hip', 'Thigh', 'Calf'])
            self.motor_temp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.motor_temp_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        signal.signal(signal.SIGINT, signal.SIG_DFL) 
        
        if not rclpy.ok():
             rclpy.init(args=None)

        self.spin_linear_speed.setValue(0.5)
        self.spin_angular_speed.setValue(0.8)

        REALSENSE_RGB_TOPIC = '/camera/front_realsense_camera/color/image_raw'
        front_cam_widget = self.label 
        
        self.robot_node = RobotControlNode(
            topic_name=REALSENSE_RGB_TOPIC, 
            label_widget=front_cam_widget
        )
        
        self.is_avoidance_enabled = False 
        self.connect_buttons()
        self.robot_node.new_frame_signal.connect(self.update_video_feed)
        
        # 🌟 [로그 연결] /rosout 구독 시그널을 GUI 업데이트 슬롯에 연결 🌟
        self.robot_node.new_log_signal.connect(self.update_log_display) 
        
        # 🌟 [LowState 연결] 배터리/온도 표시 🌟
        self.robot_node.state_update_signal.connect(self.update_robot_state_display)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(10)

    def update_robot_state_display(self, soc: int, max_temp: int, temps_12: list):
        """수신된 상태 데이터를 GUI 위젯에 표시 (QTableWidget 업데이트 포함)"""
        try:
            # 1. 배터리 잔량 (QProgressBar)
            self.battery_soc.setValue(soc) 
            
            # 2. 모터 최고 온도 (QLabel) - UI 파일에 'motor_temp' 위젯이 없으므로, 
            #    임시로 'label_2'에 표시하거나 오류를 피합니다. 여기서는 UI에 없는 위젯 사용을 피하고 오류를 출력합니다.
            # 🚨 [주의] UI에 motor_temp 위젯이 없습니다. 이 부분을 주석처리하거나, UI에 motor_temp QLabel을 추가하세요.
            # self.motor_temp.setText(f"Max Temp: {max_temp}°C") 
            
            # 3. QTableWidget 업데이트 (motor_temp_table)
            if hasattr(self, 'motor_temp_table'):
                for leg_index in range(4): 
                    for joint_index in range(3): 
                        
                        motor_array_index = (leg_index * 3) + joint_index
                        temp = temps_12[motor_array_index]
                        
                        # 🚨 [핵심] QTableWidgetItem을 생성하여 셀에 삽입
                        item = QTableWidgetItem(f"{temp}°C")
                        self.motor_temp_table.setItem(joint_index, leg_index, item)
            
        except AttributeError as e:
            # 이 오류가 계속 발생하면 UI에 위젯이 누락된 것입니다.
            pass

    # --- (나머지 함수들은 이전 코드와 동일하게 유지) ---
    def update_log_display(self, message: str):
        """수신된 /rosout 메시지를 QTextBrowser에 추가"""
        try:
            self.status_log_display.append(message) 
            self.status_log_display.ensureCursorVisible()
        except AttributeError:
            pass 

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.robot_node.destroy_node()
        rclpy.shutdown()
        event.accept()
        
    def set_speed_level(self, linear: float, angular: float):
        self.spin_linear_speed.setValue(linear)
        self.spin_angular_speed.setValue(angular)
        self.robot_node.get_logger().info(f"Speed set to Linear: {linear}, Angular: {angular}")

    def send_move_command_handler(self, vx: float, vy: float, vyaw: float):
        if self.is_avoidance_enabled:
            self.robot_node.send_avoidance_move(vx, vy, vyaw)
        else:
            self.robot_node.send_sport_command(SPORT_API_ID_MOVE, vx=vx, vy=vy, vyaw=vyaw)

    def connect_buttons(self):
        linear = lambda: self.spin_linear_speed.value()
        angular = lambda: self.spin_angular_speed.value()
        try:
            self.pushButton_6.clicked.connect(lambda: self.set_speed_level(0.25, 0.4)) 
            self.pushButton_7.clicked.connect(lambda: self.set_speed_level(0.5, 0.8))  
            self.pushButton_8.clicked.connect(lambda: self.set_speed_level(1.0, 1.6)) 
            self.btn_up.clicked.connect(lambda: self.send_move_command_handler(linear(), 0.0, 0.0))
            self.btn_down.clicked.connect(lambda: self.send_move_command_handler(-linear(), 0.0, 0.0))
            self.btn_left.clicked.connect(lambda: self.send_move_command_handler(0.0, linear(), 0.0)) 
            self.btn_right.clicked.connect(lambda: self.send_move_command_handler(0.0, -linear(), 0.0))
            self.pushButton_4.clicked.connect(lambda: self.send_move_command_handler(0.0, 0.0, angular())) 
            self.pushButton_5.clicked.connect(lambda: self.send_move_command_handler(0.0, 0.0, -angular()))
            self.pushButton.clicked.connect(lambda: self.robot_node.send_sport_command(SPORT_API_ID_BALANCESTAND)) 
            self.pushButton_2.clicked.connect(lambda: self.robot_node.send_sport_command(SPORT_API_ID_STANDUP))   
            self.pushButton_3.clicked.connect(lambda: self.robot_node.send_sport_command(SPORT_API_ID_DAMP))      
            self.checkBox.toggled.connect(self.handle_avoidance_toggle)
        except AttributeError as e:
            self.robot_node.get_logger().error(f"UI widget connection failed: {e}. Check UI objectNames.")

    def handle_avoidance_toggle(self, checked: bool):
        self.is_avoidance_enabled = checked 
        self.robot_node.set_avoidance_switch(checked) 

    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return 
        linear = self.spin_linear_speed.value()
        angular = self.spin_angular_speed.value()
        vx, vy, vyaw = 0.0, 0.0, 0.0
        if event.key() == Qt.Key_W: vx = linear
        elif event.key() == Qt.Key_S: vx = -linear
        elif event.key() == Qt.Key_A: vy = linear
        elif event.key() == Qt.Key_D: vy = -linear
        elif event.key() == Qt.Key_Q: vyaw = angular
        elif event.key() == Qt.Key_E: vyaw = -angular
        if vx != 0.0 or vy != 0.0 or vyaw != 0.0:
            self.send_move_command_handler(vx, vy, vyaw)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        if event.key() in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D, Qt.Key_Q, Qt.Key_E]:
            self.send_move_command_handler(0.0, 0.0, 0.0) 
        else:
            super().keyReleaseEvent(event)

    def ros_spin(self):
        rclpy.spin_once(self.robot_node, timeout_sec=0)

    def update_video_feed(self, pixmap):
        self.label.setPixmap(pixmap)


# ----------------------------------------------------
# 3. 애플리케이션 실행
# ----------------------------------------------------
def main(args=None):
    if not rclpy.ok():
         rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
