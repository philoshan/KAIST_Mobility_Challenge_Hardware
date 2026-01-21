import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. cav_id 입력받기 (기본값 CAV_01)
    cav_id_arg = DeclareLaunchArgument(
        'cav_id',
        default_value='CAV_01',
        description='Vehicle Namespace ID (e.g., CAV_01, CAV_02)'
    )

    cav_id = LaunchConfiguration('cav_id')

    # 2. [핵심] ID에 따라 CSV 파일 경로 자동 생성
    # 로직: 'CAV_02' -> split('_') -> '02' -> int() -> 2 -> 'tool/cav2p3.csv'
    
    # 일반 주행 경로 (tool/cavXp3.csv)
    original_path_expr = PythonExpression([
        "'tool/cav' + str(int('", cav_id, "'.split('_')[1])) + 'p3.csv'"
    ])

    # 회전교차로 내부 경로 (tool/cavXp3_inside.csv)
    inside_path_expr = PythonExpression([
        "'tool/cav' + str(int('", cav_id, "'.split('_')[1])) + 'p3_inside.csv'"
    ])

    # 3. 노드 설정
    control_node = Node(
        package='p3_cpp',
        executable='control_p3',
        namespace=cav_id,
        output='screen',
        parameters=[{
            'original_way_path': original_path_expr, # 동적으로 만든 경로 주입
            'inside_way_path': inside_path_expr,     # 동적으로 만든 경로 주입
            'target_speed': 2.0,
            'k_gain': 2.0,
            'max_steer': 0.7,
            'wheelbase': 0.33,
            'center_to_front': 0.17,
            'steer_gain': 1.0,
            'forward_step': 15,
            'warmup_steps': 10
        }]
    )

    return LaunchDescription([
        cav_id_arg,
        control_node
    ])