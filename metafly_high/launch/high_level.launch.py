from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    bird_name = LaunchConfiguration('bird_name', default='charlie_3')
    
    # Paths to the RViz config file
    rviz_basic_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'basic.rviz')
    rviz_PID_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'PID.rviz')
    rviz_switching_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'switching.rviz')
    rviz_geometric_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'geometric.rviz')
    rviz_returning_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'returning.rviz')
    rviz_drift_config_path = os.path.join(
        get_package_share_directory('metafly_high'), 'config', 'drift.rviz')

    # Path to listener.launch.py
    listener_launch_path = os.path.join(get_package_share_directory('metafly_listener'), 'launch', 'listener.launch.py')

    # Helper function to generate Node with policy condition
    def create_node(package, executable, name, condition_expression):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            condition=IfCondition(PythonExpression([f'"{condition_expression}" == "', policy, '"']))
        )

    # Nodes for different policies
    high_level_basic_node = create_node('metafly_high', 'high_level_basic', 'high_level_basic', 'basic')
    high_level_PID_node = create_node('metafly_high', 'high_level_PID', 'high_level_PID', 'PID')
    high_level_switching_node = create_node('metafly_high', 'high_level_switching', 'high_level_switching', 'switching')
    high_level_geometric_node = create_node('metafly_high', 'high_level_geometric', 'high_level_geometric', 'geometric')
    high_level_returning_node = create_node('metafly_high', 'high_level_returning', 'high_level_returning', 'returning')
    high_level_drift_node = create_node('metafly_high', 'high_level_drift', 'high_level_drift', 'drift')

    # RViz nodes with conditional configurations
    def create_rviz_node(config_path, condition_expression):
        return Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_path],
            condition=IfCondition(PythonExpression([
                f'"{condition_expression}" == "', policy, '" and "true" == "', use_high_rviz, '"'
            ]))
        )

    rviz_basic_node = create_rviz_node(rviz_basic_config_path, 'basic')
    rviz_PID_node = create_rviz_node(rviz_PID_config_path, 'PID')
    rviz_switching_node = create_rviz_node(rviz_switching_config_path, 'switching')
    rviz_geometric_node = create_rviz_node(rviz_geometric_config_path, 'geometric')
    rviz_returning_node = create_rviz_node(rviz_returning_config_path, 'returning')
    rviz_drift_node = create_rviz_node(rviz_drift_config_path, 'drift')

    # Include listener.launch.py
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
        high_level_PID_node,
        high_level_switching_node,
        high_level_geometric_node,
        high_level_returning_node,
        high_level_drift_node,
        rviz_basic_node,
        rviz_PID_node,
        rviz_switching_node,
        rviz_geometric_node,
        rviz_returning_node,
        rviz_drift_node,
    ])
