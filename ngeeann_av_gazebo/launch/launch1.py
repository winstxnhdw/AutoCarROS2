import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'empty.world'
    world = os.path.join(get_package_share_directory('ngeeann_av_gazebo'),
                         'worlds', world_file_name)
    robot_state_pub_dir = os.path.join(get_package_share_directory('ngeeann_av_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'ngeeann_av', '-topic', '/robot_description'],
                    output='screen')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_state_pub_dir, '/robot_state_pub_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        spawn_entity,        

    ])