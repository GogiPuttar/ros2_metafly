from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the policy argument
    policy = LaunchConfiguration('policy', default='basic')

    # Declare the ps3_override argument
    ps3_override = LaunchConfiguration('ps3_override', default='false')

    # Declare the use_high_rviz argument
    use_high_rviz = LaunchConfiguration('use_high_rviz', default='true')

    # Declare bird_name argument
    bird_name = LaunchConfiguration('bird_name', default='charlie_1')
    
    # Path to the RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'basic.rviz')

    # Path to listener.launch.py
    listener_launch_path = os.path.join(get_package_share_directory('metafly_listener'), 'launch', 'listener.launch.py')

    # Conditionally launch the high_level_basic node if policy is set to "basic"
    high_level_basic_node = Node(
        package='metafly_high',
        executable='high_level_basic',
        name='high_level_basic',
        output='screen',
        condition=IfCondition(EqualsSubstitution(policy, 'basic'))
    )

    # RViz node (conditionally launched if use_high_rviz is true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_high_rviz)
    )

    # Include listener.launch.py with use_rviz set to false and ps3_override passed through
    listener_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(listener_launch_path),
        launch_arguments={
            'use_rviz': 'false',
            'ps3_override': ps3_override,
            'bird_name': bird_name
        }.items()
    )

    return LaunchDescription([
        listener_launch,
        high_level_basic_node,
        rviz_node
    ])
