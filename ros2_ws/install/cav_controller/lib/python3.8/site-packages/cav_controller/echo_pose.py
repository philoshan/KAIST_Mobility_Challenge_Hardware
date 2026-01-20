#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

class EchoPose(Node):
    def __init__(self):
        super().__init__('echo_pose_node')
        
        # /Ego_pose 토픽을 구독합니다.
        # qos_profile_sensor_data: 시뮬레이터 데이터가 잘 안 받아질 때를 대비한 최적의 설정
        self.create_subscription(
            PoseStamped,
            '/Ego_pose',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("waiting for /Ego_pose data...")

    def listener_callback(self, msg):
        # 데이터가 들어올 때마다 실행되는 함수
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = msg.pose.orientation.z # 쿼터니언 단순 확인용
        
        self.get_logger().info(f"📍 현재 위치 수신중 -> X: {x:.2f}, Y: {y:.2f}, Yaw(z): {yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = EchoPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()