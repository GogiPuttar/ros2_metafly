from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # Conditionally launch the high_level_PID node if policy is set to "PID"
    high_level_PID_node = Node(
        package='metafly_high',
        executable='high_level_PID',
        name='high_level_PID',
        output='screen',
        condition=IfCondition(EqualsSubstitution(policy, 'PID'))
    )

    # Conditionally launch the high_level_switching node if policy is set to "switching"
    high_level_switching_node = Node(
        package='metafly_high',
        executable='high_level_switching',
        name='high_level_switching',
        output='screen',
        condition=IfCondition(EqualsSubstitution(policy, 'switching'))
    )

    # Conditionally launch the high_level_geometric node if policy is set to "geometric"
    high_level_geometric_node = Node(
        package='metafly_high',
        executable='high_level_geometric',
        name='high_level_geometric',
        output='screen',
        condition=IfCondition(EqualsSubstitution(policy, 'geometric'))
    )

    # RViz node with basic config (conditionally launched if use_high_rviz is true and policy is basic)
    rviz_basic_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_basic_config_path],
        condition=IfCondition(PythonExpression([
            '"basic" == "', policy, '" and "true" == "', use_high_rviz, '"'
        ]))
    )

    # RViz node with PID config (conditionally launched if use_high_rviz is true and policy is PID)
    rviz_PID_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_PID_config_path],
        condition=IfCondition(PythonExpression([
            '"PID" == "', policy, '" and "true" == "', use_high_rviz, '"'
        ]))
    )

    # RViz node with switching config (conditionally launched if use_high_rviz is true and policy is switching)
    rviz_switching_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_switching_config_path],
        condition=IfCondition(PythonExpression([
            '"switching" == "', policy, '" and "true" == "', use_high_rviz, '"'
        ]))
    )

    # RViz node with geometric config (conditionally launched if use_high_rviz is true and policy is geometric)
    rviz_geometric_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_geometric_config_path],
        condition=IfCondition(PythonExpression([
            '"geometric" == "', policy, '" and "true" == "', use_high_rviz, '"'
        ]))
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
        high_level_PID_node,
        high_level_switching_node,
        high_level_geometric_node,
        rviz_basic_node,
        rviz_PID_node,
        rviz_switching_node,
        rviz_geometric_node,
    ])
