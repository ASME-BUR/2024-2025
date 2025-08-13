import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    manager_dir = get_package_share_directory('bur_autonomy')
    params_file = os.path.join(manager_dir, 'params', 'params.yaml')
    bt_file = os.path.join(manager_dir, 'behavior_trees', 'open_loop.xml')

    print(bt_file)
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'behavior_tree': bt_file,
        },
        convert_types=True)

    ld = LaunchDescription()

    node = Node(
            package="bur_autonomy",
            executable="open_loop",
            name="open_loop",
            # parameters=[configured_params],
            parameters=[
                {"behavior_tree": bt_file}
            ],
            arguments=[],
        )

    ld.add_action(node)
    return ld
