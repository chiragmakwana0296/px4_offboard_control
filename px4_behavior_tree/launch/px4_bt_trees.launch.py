from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_behavior_tree',
            namespace='',
            executable='px4_behavior_tree',
            # Do not declare a node name otherwise it messes with the action node names and will result in
            # duplicate nodes!
            output='screen',
            parameters=[
                {'bt_file_path': os.path.join(get_package_share_directory('px4_behavior_tree'), 'trees/mission.xml')},
                {'plugins': ["node'
                             ]}
            ]
        ),
    ])