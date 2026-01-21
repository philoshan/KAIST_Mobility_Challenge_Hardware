import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p3_cpp',
            executable='main_p3',
            name='main_traffic_controller',
            output='screen',
            # 모든 차량과 통신하기 위해 Domain ID를 1로 고정 (현장 상황에 맞게 변경 가능)
            additional_env={'ROS_DOMAIN_ID': '10'} 
        )
    ])