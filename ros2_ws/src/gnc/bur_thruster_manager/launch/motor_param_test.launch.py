import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('bur_thruster_manager'),
        'config',
        'motor_force_config_NWU.yaml'
        )
        
    node=Node(
        package = 'bur_thruster_manager',
        name = 'thruster_manager',
        executable = 'thruster_manager',
        output = 'screen',
        parameters = [config,
                      {"wrench_sub_topic": "control_effort"},
                      {"cmd_sub_topic": "command"},
                      {"thrust_cmd_pub_topic": "thruster_command"},
                      {"test_mode": False}
                      ]
    )

    ld.add_action(node)
    return ld