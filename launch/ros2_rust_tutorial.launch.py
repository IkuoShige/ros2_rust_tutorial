from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    """
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Get the RViz configuration file path
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_rust_tutorial'),
        'rviz', 'ros2_rust_tutorial.rviz'
    )
    
    # Define launch actions
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set to "true" to launch RViz'
    )
    
    custom_msg_publisher_node = Node(
        package='ros2_rust_tutorial',
        executable='custom_msg_publisher',
        name='custom_msg_publisher',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(use_rviz)
    )
    
    # Compose launch description
    return LaunchDescription([
        declare_use_rviz,
        custom_msg_publisher_node,
        rviz_node
    ])