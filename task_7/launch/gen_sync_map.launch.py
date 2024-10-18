from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    # Include SLAM launch file from turtlebot4_navigation
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={'namespace': '/robot'}.items()
    )

    # Include RViz2 launch file from turtlebot4_viz
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_viz'),
                'launch',
                'view_robot.launch.py'
            )
        )
    )

    # Launch teleop_twist_keyboard node
    teleop_node = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
             ],
        output='screen'
    )

    return LaunchDescription([slam_launch, view_robot_launch, teleop_node])
