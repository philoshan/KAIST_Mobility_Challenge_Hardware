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

    # 2. [핵심] Node() 대신 ExecuteProcess() 사용
    # 터미널에서 우리가 치고 싶은 명령어를 그대로 리스트로 만듭니다.
    # 명령: ros2 run kmc_hardware_driver_node kmc_hardware_driver_observe_node --ros-args -r /Ego_pose:=/CAV_01/Ego_pose -r __ns:=/CAV_01
    
    driver_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'kmc_hardware_driver_node', 'kmc_hardware_driver_observe_node',
            '--ros-args',
            '-r', '__ns:=/',                 # [중요 1] 기본 네임스페이스를 루트(/)로 초기화 (중복 방지)
            '-r', ['/Ego_pose:=/', cav_id, '/Ego_pose'], # [중요 2] /Ego_pose를 /CAV_01/Ego_pose로 강제 매핑
            '-r', ['/cmd_vel:=/', cav_id, '/cmd_vel'],   # cmd_vel도 확실하게 매핑
            '-r', ['/vehicle_speed:=/', cav_id, '/vehicle_speed'],
            '-r', ['/battery_voltage:=/', cav_id, '/battery_voltage']
        ],
        output='screen'
    )

    return LaunchDescription([
        cav_id_arg,
        driver_process
    ])