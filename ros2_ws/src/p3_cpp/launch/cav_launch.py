import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. 인자 선언 (기본값 '01')
    id_arg = DeclareLaunchArgument(
        'id', default_value='01',
        description='CAV ID (e.g., 01, 02) - Matches config filename and ROS_DOMAIN_ID'
    )
    
    cav_id = LaunchConfiguration('id')

    # 2. Domain ID 설정 (제어 노드용)
    # domain_bridge 노드는 이 설정과 무관하게 YAML 파일 설정을 따름
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=cav_id
    )

    # 3. 경로 동적 생성
    
    # (1) CSV 파일 경로 (제어용)
    # 예: .../tool/cav01p3.csv
    original_filename = PythonExpression(["'cav' + '", cav_id, "' + 'p3.csv'"])
    inside_filename = PythonExpression(["'cav' + '", cav_id, "' + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', original_filename
    ])
    inside_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', inside_filename
    ])

    # (2) Bridge YAML 파일 경로 (통신용) [추가된 부분]
    # 예: .../config/bridge_01.yaml
    bridge_filename = PythonExpression(["'bridge_' + '", cav_id, "' + '.yaml'"])
    bridge_config_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'config', bridge_filename
    ])

    # 4. 노드 설정
    
    # [Node 1] Stanley Controller (제어기)
    control_node = Node(
        package='p3_cpp',
        executable='control_p3',
        name=['stanley_controller_', cav_id],
        namespace=['CAV_', cav_id],
        output='screen',
        parameters=[{
            'original_way_path': original_path,
            'inside_way_path': inside_path
        }],
        # 관제탑과 통신하기 위해 전역 토픽 이름으로 리매핑
        remappings=[
            ('Ego_pose', '/Ego_pose'), 
            ('cmd_vel', '/cmd_vel'),
            ('cmd_stop', '/cmd_stop')
        ]
    )

    # [Node 2] Domain Bridge (통신 브릿지)
    # 해당 차량의 ID에 맞는 YAML 파일을 로드하여 실행
    bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name=['domain_bridge_', cav_id],
        output='screen',
        arguments=[bridge_config_path]
    )

    return LaunchDescription([
        id_arg,
        set_domain_id,
        control_node,
        bridge_node
    ])