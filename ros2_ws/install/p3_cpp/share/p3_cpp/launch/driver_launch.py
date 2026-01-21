import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. cav_id 입력받기 (기본값 CAV_01)
    cav_id_arg = DeclareLaunchArgument(
        'cav_id',
        default_value='CAV_01',
        description='Vehicle Namespace ID'
    )
    
    cav_id = LaunchConfiguration('cav_id')

    # 2. [수정됨] ExecuteProcess 설정
    # 핵심: Controller가 /Ego_pose를 기다리므로, Driver도 /Ego_pose로 쏘게 만듭니다.
    
    driver_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'kmc_hardware_driver_node', 'kmc_hardware_driver_observe_node',
            '--ros-args',
            '-r', '/Ego_pose:=/Ego_pose',

            '-r', '/cmd_vel:=/cmd_vel',

            '-r', '/vehicle_speed:=/vehicle_speed',
            '-r', '/battery_voltage:=/battery_voltage'
        ],
        output='screen'
    )

    return LaunchDescription([
        cav_id_arg,
        driver_process
    ])