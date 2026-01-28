import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'p3_cpp'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 실행 인자 정의
    
    # [LID] 논리적 역할 ID (1, 2, 3, 4) 
    # -> 이 번호가 '가면(Namespace)'과 '경로(CSV)', '통신설정(YAML)'을 모두 결정합니다.
    logical_id_arg = DeclareLaunchArgument(
        'lid',
        default_value='1',
        description='Logical Role ID (1, 2, 3, 4) -> Determines Namespace /CAV_0x'
    )

    # [PID] 실제 차량 번호 (참고용/로그용)
    # 실제 통신 도메인 설정은 YAML 파일에서 직접 수정하므로, 여기서는 로직에 영향을 주지 않습니다.
    physical_id_arg = DeclareLaunchArgument(
        'pid',
        default_value='27',
        description='Physical Vehicle ID (Just for logging)'
    )

    lid = LaunchConfiguration('lid')
    pid = LaunchConfiguration('pid')

    # 2. [핵심] 네임스페이스를 역할(LID)에 맞춰 강제 설정 ("Costume Mode")
    # 예: lid가 1이면 -> "/CAV_01" (실제 차가 27번이든 4번이든 상관없음)
    vehicle_namespace = PythonExpression(["'/CAV_0' + str('", lid, "')"])

    # 3. 파일 경로 동적 생성
    
    # CSV 파일: tool/cav1p3.csv (lid 기준)
    original_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3.csv'"])
    inside_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([pkg_share, 'tool', original_csv_name])
    inside_path = PathJoinSubstitution([pkg_share, 'tool', inside_csv_name])

    # 브릿지 설정 파일: config/bridge_role_01.yaml (lid 기준)
    # 이 파일 내부의 'from_domain' 숫자만 대회 당일 차량에 맞춰 수정하면 됩니다.
    bridge_yaml_name = PythonExpression(["'bridge_role_0' + str('", lid, "') + '.yaml'"])
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', bridge_yaml_name])

    return LaunchDescription([
        logical_id_arg,
        physical_id_arg,

        LogInfo(msg=["=== Launching in Costume Mode (Role Based) ==="]),
        LogInfo(msg=["My Role (LID): ", lid]),
        LogInfo(msg=["My Mask (Namespace): ", vehicle_namespace]),
        LogInfo(msg=["Loading Bridge Config: ", bridge_yaml_name]),

        # [노드 1] Stanley 제어기
        Node(
            package=pkg_name,
            executable='control_p3',
            name='stanley_tracker',
            namespace=vehicle_namespace, # [중요] 역할 이름(/CAV_01) 방 안에서 실행!
            output='screen',
            parameters=[{
                'original_way_path': original_path,
                'inside_way_path': inside_path,
                'target_speed': 2.0,
                'k_gain': 2.0,
                'max_steer': 0.7,
                'wheelbase': 0.33,
                'center_to_front': 0.17,
                'steer_gain': 1.0,
                'forward_step': 15,
                'warmup_steps': 10
            }]
        ),

        # [노드 2] 도메인 브릿지
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            output='screen',
            arguments=[bridge_config_path]
        )
    ])