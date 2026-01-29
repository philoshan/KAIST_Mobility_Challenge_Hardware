import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 1. 인자 선언
    
    # (1) ID 및 역할 관련 인자
    # id: 물리적 장치 ID (ROS_DOMAIN_ID, Namespace, Bridge 설정용)
    id_arg = DeclareLaunchArgument(
        'id', default_value='01',
        description='Physical CAV ID (e.g., 01, 12) - Matches ROS_DOMAIN_ID and node namespace'
    )
    
    # role: 논리적 역할 (CSV 경로 파일 선택용)
    # 예: 실제 차는 12번이지만 1번 차량의 경로(cav01p3.csv)를 따라가고 싶을 때 사용
    role_arg = DeclareLaunchArgument(
        'role', default_value='01',
        description='Logical Role ID (e.g., 01, 02) - Determines which CSV path file to load'
    )

    # (2) 제어 파라미터 인자 (실행 시 튜닝 가능하도록 노출)
    k_gain_arg = DeclareLaunchArgument('k_gain', default_value='1.2')
    max_steer_arg = DeclareLaunchArgument('max_steer', default_value='0.56')
    target_speed_arg = DeclareLaunchArgument('target_speed', default_value='0.5')
    center_to_front_arg = DeclareLaunchArgument('center_to_front', default_value='0.1055')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.211')
    steer_gain_arg = DeclareLaunchArgument('steer_gain', default_value='1.0')
    forward_step_arg = DeclareLaunchArgument('forward_step', default_value='8')
    warmup_steps_arg = DeclareLaunchArgument('warmup_steps', default_value='10')

    # LaunchConfiguration 변수 매핑
    cav_id = LaunchConfiguration('id')
    role_id = LaunchConfiguration('role')
    
    k_gain = LaunchConfiguration('k_gain')
    max_steer = LaunchConfiguration('max_steer')
    target_speed = LaunchConfiguration('target_speed')
    center_to_front = LaunchConfiguration('center_to_front')
    wheelbase = LaunchConfiguration('wheelbase')
    steer_gain = LaunchConfiguration('steer_gain')
    forward_step = LaunchConfiguration('forward_step')
    warmup_steps = LaunchConfiguration('warmup_steps')


    # 2. Domain ID 설정 (제어 노드용)
    # 물리적 ID(cav_id)를 따름
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=cav_id
    )


    # 3. 경로 동적 생성 (Role 기반)
    # 중요: 경로는 역할(role_id)에 따라 결정됨
    # 예: role:=01 -> cav01p3.csv 로드
    
    original_filename = PythonExpression(["'cav' + '", role_id, "' + 'p3.csv'"])
    inside_filename = PythonExpression(["'cav' + '", role_id, "' + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', original_filename
    ])
    inside_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', inside_filename
    ])


    # (2) Bridge YAML 파일 경로 (물리적 ID 기준)
    # 통신 포트나 장비 설정은 물리적 장치(cav_id)를 따라야 함
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
        namespace=['CAV_', cav_id], # 네임스페이스는 물리적 ID 사용
        output='screen',
        parameters=[{
            'original_way_path': original_path, # 역할에 따른 경로
            'inside_way_path': inside_path,     # 역할에 따른 경로
            'k_gain': k_gain,
            'max_steer': max_steer,
            'target_speed': target_speed,
            'center_to_front': center_to_front,
            'wheelbase': wheelbase,
            'steer_gain': steer_gain,
            'forward_step': forward_step,
            'warmup_steps': warmup_steps
        }],
        # 관제탑과 통신하기 위해 전역 토픽 이름으로 리매핑
        remappings=[
            ('Ego_pose', '/Ego_pose'), 
            ('cmd_vel', '/cmd_vel'),
            ('cmd_stop', '/cmd_stop')
        ]
    )


    # [Node 2] Domain Bridge (통신 브릿지)
    # 해당 차량의 물리적 ID에 맞는 YAML 파일을 로드하여 실행
    bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name=['domain_bridge_', cav_id],
        output='screen',
        arguments=[bridge_config_path]
    )


    return LaunchDescription([
        # Args
        id_arg, role_arg,
        k_gain_arg, max_steer_arg, target_speed_arg,
        center_to_front_arg, wheelbase_arg, steer_gain_arg,
        forward_step_arg, warmup_steps_arg,
        
        # Actions
        set_domain_id,
        control_node,
        bridge_node
    ])
