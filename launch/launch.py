import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    bag_record = LaunchConfiguration('bag_record')
    ros2_gazebo_package_dir = get_package_share_directory('ros2_gazebo')
    gazebo_world_path = ros2_gazebo_package_dir + '/worlds/ros2_gazebo_world.world'

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_record',
            default_value= 'False'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),
        # Launch Gazebo with the specified world file
        # Node(
        #     package='gazebo_ros',
        #     executable='gazebo',
        #     name='gazebo',
        #     output='screen',
        #     arguments=['--verbose', '-s', 'libgazebo_ros_init.so', gazebo_world_path]
        # ),
        # Launch the ros2_gazebo_node
        Node(
            package='ros2_gazebo',
            executable='ros2_gazebo',
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(bag_record),
            cmd=['ros2', 'bag', 'record', '-a', '-x /camera.+'],
            shell=True
        )
    ])
