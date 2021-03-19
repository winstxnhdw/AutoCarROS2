import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navpkg = 'ngeeann_av_nav'
    gzpkg = 'ngeeann_av_gazebo'

    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    node=Node(
        package = navpkg,
        name = 'path_tracker',
        executable = 'tracker.py',
        parameters = [config]
    )

    node=Node(
        package = navpkg,
        name = 'global_planner',
        executable = 'globalplanner.py',
        parameters = [config]
    )

    node=Node(
        package = navpkg,
        name = 'localisation',
        executable = 'localisation.py',
    )

    node=Node(
        package = navpkg,
        name = 'bof',
        executable = 'bof.py',
    )

    node=Node(
        package = navpkg,
        name = 'local_planner',
        executable = 'localplanner.py',
        parameters = [config]
    )

    ld.add_action(node)
    return ld