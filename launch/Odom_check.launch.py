import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('autonomous_rover')

    # Include robot state publisher launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'rsp.launch.py')
        )
    )

    # Node for motor controller
    motor_controller_node = Node(
        package='autonomous_rover',
        executable='motor_cont.py',
        name='motor_controller',
        output='screen'
    )

    # Node for localization
    localization_node = Node(
        package='autonomous_rover',
        executable='loc.py',
        name='localization',
        output='screen'
    )

    # Launch Description
    return LaunchDescription([
        rsp_launch,
        motor_controller_node,
        localization_node
    ])
