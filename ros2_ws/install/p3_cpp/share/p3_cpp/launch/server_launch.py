import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    
    # [수정 1] 시스템 설계도(표)에 명시된 관제 도메인 ID 설정
    # 표 상단의 'Domain ID 100'을 준수해야 모든 차량 정보를 수집할 수 있습니다.
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='100'
    )

    # [수정 2] 메인 관제 노드 설정
    main_controller_node = Node(
        package='p3_cpp',
        executable='main_p3',
        name='main_traffic_controller',
        output='screen',
    )

    return LaunchDescription([
        set_domain_id,
        main_controller_node
    ])