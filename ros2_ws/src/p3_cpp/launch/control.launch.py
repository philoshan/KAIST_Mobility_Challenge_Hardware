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
    # [LID] 논리적 역할 ID (1, 2, 3, 4) -> 네임스페이스와 파일을 결정하는 핵심!
    logical_id_arg = DeclareLaunchArgument(
        'lid',
        default_value='1',
        description='Logical Role ID (1, 2, 3, 4) -> Determines Namespace /CAV_0x and CSV/Bridge files'
    )

    # [PID] 실제 차량 번호 (단순 확인용)
    physical_id_arg = DeclareLaunchArgument(
        'pid',
        default_value='27',
        description='Physical Vehicle ID (Set this in the YAML file!)'
    )

    lid = LaunchConfiguration('lid')
    pid = LaunchConfiguration('pid')

    # 2. [핵심] 네임스페이스 설정 (LID를 따라감)
    # 예: lid가 1이면 -> "/CAV_01" (가면 쓰기)
    vehicle_namespace = PythonExpression(["'/CAV_0' + str('", lid, "')"])

    # 3. 파일 경로 생성 (LID를 따라감)
    # CSV 파일: tool/cav1p3.csv
    original_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3.csv'"])
    inside_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([pkg_share, 'tool', original_csv_name])
    inside_path = PathJoinSubstitution([pkg_share, 'tool', inside_csv_name])

    # 브릿지 설정 파일: config/bridge_role_01.yaml
    bridge_yaml_name = PythonExpression(["'bridge_role_0' + str('", lid, "') + '.yaml'"])
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', bridge_yaml_name])

    return LaunchDescription([
        logical_id_arg,
        physical_id_arg,

        LogInfo(msg=["=== Launching Control (Costume Mode) ==="]),
        LogInfo(msg=["My Role (LID): ", lid]),
        LogInfo(msg=["Target Namespace: ", vehicle_namespace]),
        LogInfo(msg=["Using Bridge Config: ", bridge_yaml_name]),
        LogInfo(msg=["[REMINDER] Run Hardware Driver with: -r __ns:=", vehicle_namespace]),

        # [노드 1] Stanley 제어기
        Node(
            package=pkg_name,
            executable='control_p3',
            name='stanley_tracker',
            namespace=vehicle_namespace, # 역할 이름(/CAV_01) 방 안에서 실행
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