#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist  # Accel 대신 Twist 사용

# ==========================================
# 1. 수학 보조 함수들 (기존 로직 유지)
# ==========================================
def wrap_to_pi(a: float) -> float:
    """각도를 -pi ~ pi 사이로 변환"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def clamp(v: float, lo: float, hi: float) -> float:
    """값의 범위를 제한"""
    return lo if v < lo else hi if v > hi else v

def euler_from_quaternion(x, y, z, w):
    """
    [중요] 쿼터니언(x,y,z,w)을 오일러 각(Roll, Pitch, Yaw)으로 변환
    우리는 평면 주행이므로 Yaw 값만 필요합니다.
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

# ==========================================
# 2. 메인 노드 클래스
# ==========================================
class StanleyFollower(Node):
    
    # 세그먼트 파일 매핑 (필요시 수정)
    SEGMENT_ALIAS = {(1, 3): "1_3.csv"} 

    def __init__(self):
        super().__init__("stanley_follower")

        # ----- 파라미터 설정 (기존 유지) -----
        self.declare_parameter("waypoint_dir", "tool/waypoint") # CSV 파일 폴더 경로
        self.declare_parameter("route_nodes", [18, 21, 51, 46, 40, 63,34,27,31,1,3,7,9,56,59]) # 경로 노드 순서

        # 차량 물리 정보
        self.declare_parameter("speed", 0.5)          # [m/s] 목표 속도
        self.declare_parameter("wheelbase", 0.211)    # [m] 축거
        self.declare_parameter("L_front", 0.15)       # [m] 전방 주시 거리 오프셋
        self.declare_parameter("heading_lookahead", 3)

        # Stanley 제어 게인
        self.declare_parameter("k_cte", 4.5)
        self.declare_parameter("eps", 0.5)
        self.declare_parameter("k_steer", 1.8)
        self.declare_parameter("max_steer", 0.9)      # [rad] 최대 조향각

        # 경로 탐색 옵션
        self.declare_parameter("search_window", 400)
        self.declare_parameter("back_allow", 2)
        
        # 랩(Lap) 관련
        self.declare_parameter("lap_finish_margin", 5)
        self.declare_parameter("lap_finish_dist", 0.25)

        # ----- 파라미터 로드 -----
        waypoint_dir_str = str(self.get_parameter("waypoint_dir").value)
        self.route_nodes = [int(x) for x in list(self.get_parameter("route_nodes").value)]

        self.v = float(self.get_parameter("speed").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.L_front = float(self.get_parameter("L_front").value)
        self.heading_lookahead = int(self.get_parameter("heading_lookahead").value)

        self.k_cte = float(self.get_parameter("k_cte").value)
        self.eps = float(self.get_parameter("eps").value)
        self.k_steer = float(self.get_parameter("k_steer").value)
        self.max_steer = float(self.get_parameter("max_steer").value)

        self.search_window = int(self.get_parameter("search_window").value)
        self.back_allow = int(self.get_parameter("back_allow").value)
        self.lap_finish_margin = int(self.get_parameter("lap_finish_margin").value)
        self.lap_finish_dist = float(self.get_parameter("lap_finish_dist").value)

        # ----- CSV 로딩 로직 -----
        self.waypoint_dir = self._resolve_waypoint_dir(waypoint_dir_str)
        self.waypoints = self._load_route_points(self.waypoint_dir, self.route_nodes)
        
        self.last_idx = 0
        self.lap_count = 0

        self.get_logger().info(f"WP 로드 완료: {len(self.waypoints)}개 | 속도: {self.v} m/s")

        # ----- [수정됨] ROS 퍼블리셔/서브스크라이버 -----
        # 1. 제어 명령: /Accel -> /cmd_vel (Twist)
        # drive_test.py와 동일하게 설정
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        # 2. 위치 수신: /Ego_pose (PoseStamped)
        # echo_pose.py와 동일하게 설정
        self.create_subscription(
            PoseStamped, 
            "/Ego_pose", 
            self.cb, 
            qos_profile_sensor_data
        )

    # ... (경로 파일 처리 함수들은 원본 로직 그대로 유지) ...
    def _resolve_waypoint_dir(self, waypoint_dir: str) -> Path:
        p = Path(waypoint_dir)
        if p.is_absolute() and p.exists(): return p.resolve()
        candidates = []
        cwd = Path.cwd().resolve()
        for parent in [cwd] + list(cwd.parents)[:6]:
            candidates.append((parent / waypoint_dir).resolve())
        script_dir = Path(__file__).resolve().parent
        for parent in [script_dir] + list(script_dir.parents)[:8]:
            candidates.append((parent / waypoint_dir).resolve())
        for c in candidates:
            if c.exists() and c.is_dir(): return c
        # 폴더가 없으면 현재 폴더로 가정 (에러 방지용 임시 처리)
        self.get_logger().warn(f"'{waypoint_dir}' 폴더를 찾지 못해 현재 폴더를 사용합니다.")
        return cwd

    def _read_csv_points(self, csv_path: Path):
        pts = []
        if not csv_path.exists():
            return pts
        with csv_path.open("r", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2: continue
                try:
                    x, y = float(row[0]), float(row[1])
                    if math.isfinite(x) and math.isfinite(y):
                        pts.append((x, y))
                except ValueError: continue
        return pts

    def _segment_csv_name(self, a: int, b: int) -> str:
        return self.SEGMENT_ALIAS.get((a, b), f"{a}_{b}.csv")

    def _load_route_points(self, waypoint_dir: Path, route_nodes):
        all_pts = []
        for i in range(len(route_nodes) - 1):
            name = self._segment_csv_name(route_nodes[i], route_nodes[i+1])
            pts = self._read_csv_points(waypoint_dir / name)
            if not pts:
                self.get_logger().warn(f"CSV 없음 또는 비었음: {name}")
                continue
            # 이어붙이기 (중복 제거)
            if all_pts:
                lx, ly = all_pts[-1]
                fx, fy = pts[0]
                if (lx-fx)**2 + (ly-fy)**2 < 1e-12:
                    pts = pts[1:]
            all_pts.extend(pts)
        
        if not all_pts:
            # 테스트를 위해 데이터가 없을 경우 가상의 직선 경로 생성
            self.get_logger().error("유효한 웨이포인트가 없어 가상 경로(0,0 -> 10,0)를 생성합니다.")
            return [(x*0.1, 0.0) for x in range(100)]
            
        return all_pts

    # ... (Geometry 로직 유지) ...
    def _nearest_index(self, x, y, start_idx):
        n = len(self.waypoints)
        i0 = max(0, start_idx - self.back_allow)
        i1 = min(n - 1, start_idx + self.search_window)
        best_i = i0
        best_d2 = float("inf")
        for i in range(i0, i1 + 1):
            wx, wy = self.waypoints[i]
            d2 = (wx - x)**2 + (wy - y)**2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _path_heading(self, idx):
        n = len(self.waypoints)
        idx = max(0, min(idx, n - 2))
        x1, y1 = self.waypoints[idx]
        x2, y2 = self.waypoints[idx + 1]
        return math.atan2(y2 - y1, x2 - x1)

    def _signed_cte_to_segment(self, idx, x, y):
        n = len(self.waypoints)
        idx = max(0, min(idx, n - 2))
        x0, y0 = self.waypoints[idx]
        x1, y1 = self.waypoints[idx+1]
        sx, sy = x1-x0, y1-y0
        seg_len2 = sx*sx + sy*sy
        if seg_len2 < 1e-12: return 0.0
        t = ((x-x0)*sx + (y-y0)*sy) / seg_len2
        t = clamp(t, 0.0, 1.0)
        cx, cy = x0 + t*sx, y0 + t*sy
        cross_z = sx*(y-y0) - sy*(x-x0)
        dist = math.hypot(x-cx, y-cy)
        return -dist if cross_z > 0.0 else dist

    def _try_wrap_lap(self, xf, yf, idx):
        n = len(self.waypoints)
        if n < 2: return idx
        if idx < (n - 1 - self.lap_finish_margin): return idx
        lx, ly = self.waypoints[-1]
        if math.hypot(lx - xf, ly - yf) > self.lap_finish_dist: return idx
        
        self.lap_count += 1
        self.last_idx = 0
        self.get_logger().info(f"🏁 랩 완료! 다시 시작 (Lap: {self.lap_count})")
        return self._nearest_index(xf, yf, 0)

    # ==========================================
    # 3. 메인 콜백 함수 (핵심 수정 부분)
    # ==========================================
    def cb(self, msg: PoseStamped):
        # 1. 위치 정보 추출
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        
        # [수정됨] 쿼터니언 -> Yaw 변환
        # 기존: yaw = msg.pose.orientation.z (잘못된 방식)
        q = msg.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # 2. 앞바퀴 중심점(Look ahead) 계산
        xf = cx + self.L_front * math.cos(yaw)
        yf = cy + self.L_front * math.sin(yaw)

        # 3. 가장 가까운 웨이포인트 찾기
        idx = self._nearest_index(xf, yf, self.last_idx)
        if idx < self.last_idx - self.back_allow:
            idx = self.last_idx - self.back_allow
        self.last_idx = idx

        # 4. 랩 반복 체크
        idx = self._try_wrap_lap(xf, yf, idx)
        self.last_idx = idx

        # 5. Stanley 알고리즘 계산
        path_yaw = self._path_heading(idx + self.heading_lookahead)
        heading_err = wrap_to_pi(path_yaw - yaw)
        
        cte = self._signed_cte_to_segment(idx, xf, yf)
        cte_term = math.atan2(self.k_cte * cte, (self.v + self.eps))
        
        stanley_angle = wrap_to_pi(heading_err + cte_term)
        
        # 조향각 제한
        delta_raw = self.k_steer * stanley_angle
        delta_cmd = clamp(delta_raw, -self.max_steer, +self.max_steer)

        # 6. 자전거 모델: 조향각(delta) -> 회전속도(omega) 변환
        # omega = (v / L) * tan(delta)
        omega_cmd = (self.v / self.wheelbase) * math.tan(delta_cmd)

        # 7. [수정됨] 명령 발행 (/cmd_vel)
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.v)   # 앞으로 가는 속도
        cmd_msg.angular.z = float(omega_cmd) # 회전 속도
        
        self.pub_cmd.publish(cmd_msg)

        # 로그 출력 (디버깅용)
        # self.get_logger().info(f"Idx:{idx} CTE:{cte:.2f} Delta:{math.degrees(delta_cmd):.1f}deg")

def main(args=None):
    rclpy.init(args=args)
    node = StanleyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령
        stop_msg = Twist()
        node.pub_cmd.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()