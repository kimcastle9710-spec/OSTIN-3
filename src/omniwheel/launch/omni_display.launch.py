# launch/omni_display.launch.py 예시 (ROS 2)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    pkg_description = get_package_share_directory('omniwheel')

    # 1. URDF 파일 경로 설정 (사용하려는 새 URDF 이름으로 변경)
    urdf_file_name = 'ostin_3.urdf' # <--- 여기에 새 파일 이름을 넣으세요
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. 로봇 상태 퍼블리셔 (RSP)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. 조인트 상태 퍼블리셔 (GUI로 관절을 수동 조작할 수 있게 해줍니다)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # 4. Rviz2 시각화 도구
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path] # Rviz 설정 파일 경로
    )

    return LaunchDescription([
        jsp_gui_node,
        rsp_node,
        rviz_node,
    ])
