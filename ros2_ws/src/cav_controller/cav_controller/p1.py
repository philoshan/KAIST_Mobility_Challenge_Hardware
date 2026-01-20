#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

import math
import csv
import sys
import os

class RealStanleyNode(Node):
    def __init__(self):
        super().__init__('real_stanley_node')

        # === [파라미터 설정] ===
        # 실제 하드웨어 주행에 맞춰 최적화된 값입니다.
        self.declare_parameter("csv_path", "tool/cav1p1.csv")
        self.declare_parameter("target_speed", 0.5)      # [m/s] 초기 안전 속도
        self.declare_parameter("wheelbase", 0.17)         # [m] 회전 반응성을 위해 설정한 값
        
        # Stanley 제어 게인
        self.declare_parameter("k_gain", 1.3)            # 경로 복귀 감도
        self.declare_parameter("max_steer", 0.7)         # [rad] 최대 조향각
        self.declare_parameter("steer_gain", 1.0)        # 조향 증폭 비율
        
        # 경로 탐색
        self.declare_parameter("center_to_front", 0.15)  # [m] 중심-전륜 거리
        self.declare_parameter("forward_step", 5)        # [idx] 주시 거리 (Lookahead index)
        self.declare_parameter("warmup_steps", 10)       # 초기 안정화 카운트

        # 파라미터 변수 로드
        self.csv_path = self.get_parameter("csv_path").value
        self.target_speed = self.get_parameter("target_speed").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.k_gain = self.get_parameter("k_gain").value
        self.max_steer = self.get_parameter("max_steer").value
        self.steer_gain = self.get_parameter("steer_gain").value
        self.center_to_front = self.get_parameter("center_to_front").value
        self.forward_step = self.get_parameter("forward_step").value
        self.warmup_steps_target = self.get_parameter("warmup_steps").value

        self.current_warmup_count = 0
        self.stop_signal = False
        self.waypoints = []

        # === [CSV 로드] ===
        self.load_waypoints(self.csv_path)

        # === [ROS 퍼블리셔/서브스크라이버] ===
        # 1. 위치 수신 (PoseStamped) - QoS Best Effort
        self.create_subscription(
            PoseStamped,
            '/Ego_pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        # 2. 제어 명령 발송 (Twist) -> /cmd_vel
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. 비상 정지 수신 (Bool) -> cmd_stop
        # main_p1.cpp와 연동하기 위해 필요
        self.create_subscription(
            Bool,
            'cmd_stop',
            self.stop_cmd_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f"🚀 Real Stanley Started! Speed: {self.target_speed}m/s")

    def load_waypoints(self, path):
        """CSV 파일에서 웨이포인트 (x, y) 로드"""
        try:
            # 절대 경로가 아니면 현재 위치 기준으로 탐색
            if not os.path.isabs(path):
                path = os.path.join(os.getcwd(), path)
            
            with open(path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) # 헤더 건너뛰기
                for row in reader:
                    if len(row) >= 2:
                        try:
                            self.waypoints.append((float(row[0]), float(row[1])))
                        except ValueError:
                            continue
            self.get_logger().info(f"✅ Waypoints loaded: {len(self.waypoints)} points from {path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load CSV: {e}")
            # 비상용 가상 경로 생성
            self.waypoints = [(x*0.1, 0.0) for x in range(100)]

    def stop_cmd_callback(self, msg):
        """외부(main_p1)에서 정지 명령이 오면 수신"""
        self.stop_signal = msg.data
        if self.stop_signal:
            self.publish_stop()

    def publish_stop(self):
        """즉시 정지 명령 전송"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 사이로 변환"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def pose_callback(self, msg):
        # 1. Warm-up (초기 안정화)
        if self.current_warmup_count < self.warmup_steps_target:
            self.current_warmup_count += 1
            self.publish_stop()
            if self.current_warmup_count == 1:
                self.get_logger().info("🔥 Warming up system...")
            return

        # 2. 비상 정지 체크
        if self.stop_signal:
            self.publish_stop()
            return

        if not self.waypoints:
            return

        # 3. 현재 위치 및 Yaw 추출 [수정됨: z값 직접 사용]
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        
        # [수정] 쿼터니언 변환 제거 -> z 값을 바로 Yaw 각도로 사용
        cyaw = msg.pose.orientation.z

        # 4. 전륜 중심점(Front Axle) 계산
        fx = cx + self.center_to_front * math.cos(cyaw)
        fy = cy + self.center_to_front * math.sin(cyaw)

        # 5. 가장 가까운 웨이포인트 찾기
        min_dist = float('inf')
        nearest_idx = -1
        for i, (wx, wy) in enumerate(self.waypoints):
            d = math.hypot(fx - wx, fy - wy)
            if d < min_dist:
                min_dist = d
                nearest_idx = i

        # 6. CTE (횡방향 오차) 계산
        next_idx = (nearest_idx + 1) % len(self.waypoints)
        wp_curr = self.waypoints[nearest_idx]
        wp_next = self.waypoints[next_idx]

        dx = wp_next[0] - wp_curr[0]
        dy = wp_next[1] - wp_curr[1]
        path_len = math.hypot(dx, dy)
        
        if path_len < 1e-6:
            cte = 0.0
        else:
            # 벡터 외적을 이용한 거리 계산
            cte = ((fx - wp_curr[0]) * dy - (fy - wp_curr[1]) * dx) / path_len

        # 7. Heading Error (주시 거리 적용)
        target_idx = (nearest_idx + self.forward_step) % len(self.waypoints)
        next_target_idx = (target_idx + 1) % len(self.waypoints)
        
        t_curr = self.waypoints[target_idx]
        t_next = self.waypoints[next_target_idx]
        
        path_yaw = math.atan2(t_next[1] - t_curr[1], t_next[0] - t_curr[0])
        heading_err = self.normalize_angle(path_yaw - cyaw)

        # 8. Stanley 제어 법칙
        # v가 0일 때 에러 방지용 최소값
        v_ref = max(self.target_speed, 0.1)
        cte_term = math.atan2(self.k_gain * cte, v_ref)

        steer_angle = heading_err + cte_term
        steer_angle = self.normalize_angle(steer_angle)
        steer_angle *= self.steer_gain

        # 조향각 제한 (Saturation)
        steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle))

        # 9. [핵심] 조향각(rad) -> 각속도(rad/s) 변환
        # Kinematic Bicycle Model: omega = (v / L) * tan(delta)
        yaw_rate = (self.target_speed / self.wheelbase) * math.tan(steer_angle)

        # 10. 명령 발행 (/cmd_vel)
        cmd = Twist()
        cmd.linear.x = float(self.target_speed)
        cmd.angular.z = float(yaw_rate)
        
        self.pub_cmd.publish(cmd)
        
        # 디버깅용 주석
        # self.get_logger().info(f"idx:{nearest_idx} cte:{cte:.2f} head:{heading_err:.2f} steer:{steer_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RealStanleyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()