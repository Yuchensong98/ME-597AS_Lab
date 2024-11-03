from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    task_7_pkg_dir = get_package_share_directory('task_7')
    turtlebot4_navigation_pkg_dir = get_package_share_directory('turtlebot4_navigation')
    turtlebot4_viz_pkg_dir = get_package_share_directory('turtlebot4_viz')

    # Specify the path to the map file
    map_file_path = os.path.join(task_7_pkg_dir, 'maps', 'classroom_map.yaml')

    # Include the localization.launch.py from turtlebot4_navigation with the map parameter
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_navigation_pkg_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'map': map_file_path}.items(),
    )

    # Include the view_robot.launch.py from turtlebot4_viz
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_viz_pkg_dir, 'launch', 'view_robot.launch.py')
        )
    )

    # Node for running the auto_navigator from task_7
    auto_navigator_node = Node(
        package='task_7',
        executable='auto_navigator',
        output='screen',
    )

    return LaunchDescription([
        localization_launch,
        view_robot_launch,
        # auto_navigator_node,
    ])
