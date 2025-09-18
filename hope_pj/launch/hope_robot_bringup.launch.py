import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    hope_pj_pkg_dir = get_package_share_directory('hope_pj')
    ydlidar_pkg_dir = get_package_share_directory('ydlidar_ros2_driver')

    # URDF 파일 경로 지정 (.xacro)
    urdf_file_path = os.path.join(hope_pj_pkg_dir, 'urdf', 'hope_robot.urdf.xacro')

    # 파라미터 파일 경로 지정
    hope_robot_params_path = os.path.join(hope_pj_pkg_dir, 'config', 'burger.yaml')
    ydlidar_params_path = os.path.join(ydlidar_pkg_dir, 'params', 'G6.yaml')

    # robot_state_publisher (xacro 처리)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file_path])
        }]
    )

    # turtlebot3_node (name, namespace 지정)
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        name='turtlebot3_node',
        namespace='',
        parameters=[hope_robot_params_path],
        output='screen',
    )

    # YDLiDAR 노드
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        robot_state_publisher_node,
        turtlebot3_node,
        ydlidar_node,
    ])
