import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='100'
    )

    main_controller_node = Node(
        package='p3_cpp',
        executable='main_p3',  # CMakeLists.txt의 설정과 일치
        name='main_traffic_controller',
        output='screen',
    )

    return LaunchDescription([
        set_domain_id,
        main_controller_node
    ])