import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    trajectory_control_server_node = Node(
        package='trajectory_control_server',
        executable='trajectory_control_server',
        namespace='',
        output='screen',
        # parameters=[os.path.join(get_package_share_directory("trajectory_control_server"), "config", 'trajectory_control_server.yaml')],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    return launch.LaunchDescription([
        trajectory_control_server_node,
    ])

if __name__ == '__main__':
    launch_description = generate_launch_description()
    launch.launch(launch_description)