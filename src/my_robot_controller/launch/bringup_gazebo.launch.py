from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch Gazebo Turtlebot3 world
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Your controller node (publishes /cmd_vel)
        Node(
            package='my_robot_controller',
            executable='simple_cmd',
            output='screen'
        ),
    ])
