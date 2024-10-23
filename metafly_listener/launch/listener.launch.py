from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare the use_rviz argument
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Declare ps3_override argument
    ps3_override = LaunchConfiguration('ps3_override', default='true')

    # Declare bird_name argument
    bird_name = LaunchConfiguration('bird_name', default='charlie_3')
    
    # Path to the RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('metafly_listener'), 'config', 'tracking_config.rviz')

    # Path to launch files
    ps3_launch_path = os.path.join(get_package_share_directory('metafly_control'), 'launch', 'ps3.launch.xml')
    controller_launch_path = os.path.join(get_package_share_directory('metafly_control'), 'launch', 'controller.launch.xml')

    # Listener node
    listener_node = Node(
        package='metafly_listener',
        executable='listener_node.py',
        name='udp_listener',
        output='screen',
        parameters=[{'bird_name': bird_name}]
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # Conditional PS3 or Controller launch based on ps3_override
    ps3_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ps3_launch_path),
        condition=IfCondition(ps3_override)
    )

    controller_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(controller_launch_path),
        condition=UnlessCondition(ps3_override)
    )

    return LaunchDescription([
        listener_node,
        rviz_node,
        ps3_launch,
        controller_launch
    ])
