import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navpkg = 'ngeeann_av_nav'
    gzpkg = 'ngeeann_av_gazebo'

    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    path_tracker=Node(
        package = navpkg,
        name = 'path_tracker',
        executable = 'tracker.py',
        parameters = [config]
    )

    global_planner=Node(
        package = navpkg,
        name = 'global_planner',
        executable = 'globalplanner.py',
        parameters = [config]
    )

    localisation=Node(
        package = navpkg,
        name = 'localisation',
        executable = 'localisation.py',
        parameters = [config]
    )

    local_planner=Node(
        package = navpkg,
        name = 'local_planner',
        executable = 'localplanner.py',
        parameters = [config]
    )

    ld.add_action(localisation)
    ld.add_action(global_planner)
    ld.add_action(local_planner)
    ld.add_action(path_tracker)

    return ld