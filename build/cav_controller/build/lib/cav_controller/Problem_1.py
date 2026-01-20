#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class Problem1(Node):
    def __init__(self):
        super().__init__("problem_1")

        # =========================
        # 테스트 파라미터
        # =========================
        self.declare_parameter("speed", 1.0)          # m/s
        self.declare_parameter("steer", 0.0)          # rad
        self.declare_parameter("wheelbase", 0.211)    # m

        self.v = float(self.get_parameter("speed").value)
        self.delta = float(self.get_parameter("steer").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)

        # =========================
        # QoS: 반드시 Reliable
        # =========================
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos
        )

        # 50Hz → SDK 예제와 동일
        self.timer = self.create_timer(1.0 / 50.0, self.timer_cb)

        self.get_logger().info(
            "[CMD_VEL TEST MODE]\n"
            f"  speed={self.v:.2f} m/s\n"
            f"  steer={self.delta:.3f} rad ({math.degrees(self.delta):.1f} deg)\n"
            "  QoS=RELIABLE, rate=50Hz"
        )

    def timer_cb(self):
        if abs(self.delta) < 1e-6:
            omega = 0.0
        else:
            omega = (self.v / self.wheelbase) * math.tan(self.delta)

        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(omega)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Problem1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
