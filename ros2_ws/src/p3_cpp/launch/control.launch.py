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
    # pid: 실제 차량 도메인 (로그 표시용, 추후 확장용)
    physical_id_arg = DeclareLaunchArgument(
        'pid',
        default_value='25',
        description='Physical Vehicle ID (actual Jetson domain: 25, 4, 27, 3)'
    )

    # lid: 논리적 역할 ID (1, 2, 3, 4) -> 이 번호로 CSV와 YAML 파일을 찾습니다.
    logical_id_arg = DeclareLaunchArgument(
        'lid',
        default_value='1',
        description='Logical Vehicle ID for CSV and Bridge File (1, 2, 3, 4)'
    )

    pid = LaunchConfiguration('pid')
    lid = LaunchConfiguration('lid')

    # 2. 파일 경로 동적 생성
    
    # CSV 파일: tool/cav1p3.csv (lid 기준)
    original_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3.csv'"])
    inside_csv_name = PythonExpression(["'cav' + str('", lid, "') + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([pkg_share, 'tool', original_csv_name])
    inside_path = PathJoinSubstitution([pkg_share, 'tool', inside_csv_name])

    # [수정됨] 브릿지 설정 파일: config/domain_1.yaml (lid 기준)
    # 이제 lid가 1이면 config/domain_1.yaml을 불러옵니다.
    bridge_yaml_name = PythonExpression(["'domain_' + str('", lid, "') + '.yaml'"])
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', bridge_yaml_name])

    return LaunchDescription([
        physical_id_arg,
        logical_id_arg,

        LogInfo(msg=["Starting Control & Bridge..."]),
        LogInfo(msg=["Physical Domain (Set in YAML): ", pid]),
        LogInfo(msg=["Logical Role (Files): ", lid]),

        # [노드 1] Stanley 제어기
        Node(
            package=pkg_name,
            executable='control_p3',
            name='stanley_tracker',
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