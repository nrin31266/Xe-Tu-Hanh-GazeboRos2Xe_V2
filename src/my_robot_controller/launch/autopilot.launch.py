from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    world = '/home/nguyenvanrin/ros2_ws/src/my_world/worlds/highway.world'
    car_sdf = '/home/nguyenvanrin/ros2_ws/src/my_car_description/models/my_car/model.sdf'

    model_path = ':'.join([
        '/home/nguyenvanrin/ros2_ws/src/my_car_description/models',
        '/usr/share/gazebo-11/models',
    ])

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_car',
            '-file', car_sdf,
            '-x', '55.0', '-y', '0.0', '-z', '0.25',
            '-Y', '0.0'
        ],
        output='screen'
    )

    avoid_node = Node(
        package='my_robot_controller',
        executable='avoid_obstacle',
        name='avoid_obstacle',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    finish_node = Node(
        package='my_robot_controller',
        executable='finish_detector',
        name='finish_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        gazebo,

        # đợi Gazebo lên rồi spawn xe
        TimerAction(period=5.0, actions=[spawn_car]),

        avoid_node,
        finish_node,
    ])
