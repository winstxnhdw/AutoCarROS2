import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess

def generate_launch_description():

    navpkg = 'autocar_nav'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'
    mappkg = 'autocar_map'

    pkg_gazebo = get_package_share_directory('gazebo_ros')

    world = os.path.join(get_package_share_directory(gzpkg), 'worlds', 'autocar.world')
    urdf = os.path.join(get_package_share_directory(descpkg),'urdf', 'autocar.xacro')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    
    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    gzserver = os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
    gzclient = os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver), launch_arguments={'world': world}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient)
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output='screen'
        ),

        Node(
            package = navpkg,
            name = 'localisation',
            executable = 'localisation.py',
            parameters = [navconfig]
        ),

        Node(
            package = mappkg,
            name = 'bof',
            executable = 'bof',
        ),

        Node(
            package = navpkg,
            name = 'click_planner',
            executable = 'clickplanner.py',
            parameters = [navconfig]
        ),

        Node(
            package = navpkg,
            name = 'path_tracker',
            executable = 'tracker.py',
            parameters = [navconfig]
        )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()