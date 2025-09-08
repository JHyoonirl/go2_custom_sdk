#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

class SportPublisher(Node):
    def __init__(self, mode: str):
        super().__init__('sport_publisher')
        self.publisher_ = self.create_publisher(Request, '/api/sport/request', 10)

        # 메시지 생성
        msg = Request()
        msg.header.identity.api_id = self.get_api_id(mode)
        msg.parameter = ""  # 이 동작들은 추가 parameter 필요 없음
        self.publisher_.publish(msg)

        self.get_logger().info(f"Sent command: {mode}")

    def get_api_id(self, mode: str) -> int:
        """
        동작 이름을 api_id 숫자로 매핑
        """
        mapping = {
            "damp": 1001,          # ROBOT_SPORT_API_ID_DAMP
            "balancestand": 1002,  # ROBOT_SPORT_API_ID_BALANCESTAND
            "standup": 1004        # ROBOT_SPORT_API_ID_STANDUP
        }
        if mode.lower() not in mapping:
            raise ValueError(f"Unknown mode: {mode}, choose from {list(mapping.keys())}")
        return mapping[mode.lower()]

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package> send_sport_commands.py <damp|balancestand|standup>")
        return

    rclpy.init()
    node = SportPublisher(sys.argv[1])
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
