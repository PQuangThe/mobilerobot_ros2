import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node

def generate_launch_description():
    # Path
    bt_xml_dir = os.path.join(get_package_share_directory('behavior_tree_demo_ros2'), 'behavior_tree_xml')

    # Parameters
    bt_xml = LaunchConfiguration('bt_xml', default=bt_xml_dir+'/demo_behavior.xml')

    behavior_tree = Node(
        package='behavior_tree_demo_ros2',
        executable='bt_main',
        parameters=[{'bt_xml': bt_xml}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(behavior_tree)
    return ld