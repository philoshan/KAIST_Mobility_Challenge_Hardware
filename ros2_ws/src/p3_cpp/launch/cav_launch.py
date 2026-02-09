import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. 인자 선언
    
    # (1) ID 및 역할 관련 인자
    # id: 물리적 장치 ID (실제 통신용 DOMAIN ID 설정에 사용)
    # 예: 대회 당일 랜덤으로 배정받는 숫자 (12, 34 등)
    id_arg = DeclareLaunchArgument(
        'id', default_value='03',
        description='Physical CAV ID (Sets ROS_DOMAIN_ID)'
    )
    
    # role: 논리적 역할 (네임스페이스 및 CSV 경로 파일 선택용)
    # 예: cav1, cav2, cav3, cav4 (관제탑에서 정해준 역할)
    # 이 값을 'cav1' 처럼 입력받도록 설정 (숫자만 입력받는 경우라면 아래 로직 수정 필요)
    role_arg = DeclareLaunchArgument(
        'role', default_value='cav3',
        description='Logical Role Name (e.g., cav1, cav2) - Sets Namespace and CSV path'
    )

    # (2) 제어 파라미터 인자
    k_gain_arg = DeclareLaunchArgument('k_gain', default_value='0.3')
    max_steer_arg = DeclareLaunchArgument('max_steer', default_value='0.56')
    target_speed_arg = DeclareLaunchArgument('target_speed', default_value='1.0')
    center_to_front_arg = DeclareLaunchArgument('center_to_front', default_value='0.1055')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.211')
    steer_gain_arg = DeclareLaunchArgument('steer_gain', default_value='0.8')
    forward_step_arg = DeclareLaunchArgument('forward_step', default_value='8')
    warmup_steps_arg = DeclareLaunchArgument('warmup_steps', default_value='10')

    # LaunchConfiguration 변수 매핑
    cav_id = LaunchConfiguration('id')
    role_name = LaunchConfiguration('role')
    
    k_gain = LaunchConfiguration('k_gain')
    max_steer = LaunchConfiguration('max_steer')
    target_speed = LaunchConfiguration('target_speed')
    center_to_front = LaunchConfiguration('center_to_front')
    wheelbase = LaunchConfiguration('wheelbase')
    steer_gain = LaunchConfiguration('steer_gain')
    forward_step = LaunchConfiguration('forward_step')
    warmup_steps = LaunchConfiguration('warmup_steps')


    # 2. Domain ID 설정 (물리적 ID 사용)
    # 실제 장비의 통신 채널을 맞추기 위함
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value=cav_id
    )


    # 3. 경로 동적 생성 (Role 기반)
    # 예: role:='cav1' -> cav1p3.csv 로드 (역할 이름이 파일명에 포함된다고 가정)
    # 만약 파일명이 cav01p3.csv 처럼 숫자라면, role 입력시 '01'로 받거나 여기서 변환 로직 필요.
    # 여기서는 role 입력값이 'cav1'이면 -> 'cav1p3.csv'를 찾는 로직으로 작성됨.
    
    # PythonExpression을 사용하여 문자열 조합
    # 파일명이 'cav1p3.csv' 형태라면 role_name 그대로 사용
    original_filename = PythonExpression(["'", role_name, "' + 'p3.csv'"])
    inside_filename = PythonExpression(["'", role_name, "' + 'p3_inside.csv'"])

    original_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', original_filename
    ])
    inside_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'tool', inside_filename
    ])


    # 4. 노드 설정
    # [Node 1] Stanley Controller (제어기)
    control_node = Node(
        package='p3_cpp',
        executable='control_p3',
        name=['stanley_controller_', role_name], # 이름도 역할 기반으로 (디버깅 용이)
        namespace=role_name,                     # [핵심] 네임스페이스를 역할(cav1 등)로 설정
        output='screen',
        parameters=[{
            'original_way_path': original_path, 
            'inside_way_path': inside_path,     
            'k_gain': k_gain,
            'max_steer': max_steer,
            'target_speed': target_speed,
            'center_to_front': center_to_front,
            'wheelbase': wheelbase,
            'steer_gain': steer_gain,
            'forward_step': forward_step,
            'warmup_steps': warmup_steps
        }],
        # 리매핑 설정
        # 1. /Ego_pose, /cmd_vel: 시뮬레이터 절대 토픽이므로 그대로 둠

        remappings=[
            ('Ego_pose', '/Ego_pose'), 
            ('cmd_vel', '/cmd_vel') # 필요시 주석 해제 (보통 로컬 cmd_vel 사용)
        ]
    )


    return LaunchDescription([
        # Args
        id_arg, role_arg,
        k_gain_arg, max_steer_arg, target_speed_arg,
        center_to_front_arg, wheelbase_arg, steer_gain_arg,
        forward_step_arg, warmup_steps_arg,
        
        # Actions
        set_domain_id,
        control_node
    ])
