from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir = FindPackageShare("quaro_description")
    urdf_path = PathJoinSubstitution([pkg_dir, "urdf", "quaro.xacro"])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
            }])

    ld.add_action(robot_state_publisher_node)

    default_rviz_config_path = PathJoinSubstitution(["quaro_description", 'rviz', 'urdf.rviz'])
    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))
    return ld

