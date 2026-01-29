from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    main_node = Node(
        package='p3_cpp',
        executable='main_p3',
        name='main_traffic_controller',
        output='screen',
        # main 코드는 현재 파라미터를 생성자에서 하드코딩 중이므로
        # 별도의 parameters 설정이 필요 없습니다. 
        # 만약 추후에 코드에 declare_parameter를 추가한다면 여기에 적으면 됩니다.
    )

    return LaunchDescription([
        main_node
    ])