import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Domain ID 설정 (관제탑 = 100)
    # 터미널에서 export 안 해도 되도록 여기서 강제 설정
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='100'
    )

    # 2. 도메인 브릿지 설정 파일 경로 찾기
    # (패키지 내 config 폴더에 domain_manage.yaml이 있다고 가정)
    bridge_config_path = PathJoinSubstitution([
        FindPackageShare('p3_cpp'), 'config', 'domain_manage.yaml'
    ])

    # 3. 관제탑 노드 (Traffic Controller)
    main_node = Node(
        package='p3_cpp',
        executable='main_p3',
        name='main_traffic_controller',
        output='screen',

        parameters=[{
            'role_cav1': 'CAV_05',  # 현재 테스트 중인 5번 차량을 cav1으로 지정
            'role_cav2': 'CAV_02',  # (필요시 수정)
            'role_cav3': 'CAV_03',  # (필요시 수정)
            'role_cav4': 'CAV_04'   # (필요시 수정)
        }]
    )

    # 4. 도메인 브릿지 노드
    # 관제탑 하나에서 1,2,3,4번 차량과 모두 통신하기 위해 실행
    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_main',
        output='screen',
        arguments=[bridge_config_path]
    )

    return LaunchDescription([
        set_domain_id,
        main_node,
        domain_bridge_node
    ])
