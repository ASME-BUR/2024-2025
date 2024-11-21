from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from glob import glob

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('bur_gz'),
                 'urdf', 'bur.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'bur', '-allow_renaming', 'true'],
    )

    world_file = os.path.join('share', 'bur_gz'), glob('worlds/demo_world.sdf')

    thruster1_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster1/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster2_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster2/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster3_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster3/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster4_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster4/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster5_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster5/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster6_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster6/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster7_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster7/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    thruster8_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/bur/joint/thruster8/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'
        ],
        output='screen'
    )
    imu_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 src/bur_gz/worlds/demo_world.sdf'])]),
        node_robot_state_publisher,
        gz_spawn_entity,

        # Thruster bridges
        thruster1_bridge,
        thruster2_bridge,
        thruster3_bridge,
        thruster4_bridge,
        thruster5_bridge,
        thruster6_bridge,
        thruster7_bridge,
        thruster8_bridge,
        imu_bridge,

        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
