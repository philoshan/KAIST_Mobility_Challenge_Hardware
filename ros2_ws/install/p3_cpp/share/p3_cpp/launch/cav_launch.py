import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 실행 시 'id' 인자를 받음 (예: ros2 launch ... id:=01)
    id_arg = DeclareLaunchArgument(
        'id', default_value='01',
        description='CAV ID (e.g., 01, 02, 03, 04)'
    )
    
    cav_id = LaunchConfiguration('id')
    pkg_share = get_package_share_directory('p3_cpp')

    # 2. Stanley 제어 노드
    control_node = Node(
        package='p3_cpp',
        executable='control_p3',
        name=['stanley_controller_', cav_id],
        namespace=['CAV_', cav_id],
        output='screen',
        parameters=[{
            'original_way_path': [os.path.join(pkg_share, 'tool', 'cav'), cav_id, 'p3.csv'],
            'inside_way_path': [os.path.join(pkg_share, 'tool', 'cav'), cav_id, 'p3_inside.csv']
        }],
        # [핵심] C++에서 'Ego_pose'라고 불렀으니, 여기서 '/Ego_pose'로 연결!
        remappings=[
            ('Ego_pose', '/Ego_pose'), 
            ('cmd_vel', '/cmd_vel'),
            ('cmd_stop', 'cmd_stop')
        ]
        # [삭제] additional_env={'ROS_DOMAIN_ID': '1'}  <-- 삭제 완료!
    )

    return LaunchDescription([
        id_arg,
        control_node
    ])