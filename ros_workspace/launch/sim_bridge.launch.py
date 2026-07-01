from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_filename',
        default_value=TextSubstitution(text='../config/ros/rod.yaml'),
        description='Absolute path to the simulation config file.'
    )

    # the sim_bridge node
    sim_bridge_node = Node(
        package='sim_bridge',
        executable='sim_bridge_node',
        name='sim_bridge',
        output='screen',
        remappings=[
        ],
        parameters=[
            {"num_rod_frames": 30},          # number of frames sampled along the rod (these are not the rod nodes, but interpolated)
            {"publish_rate_hz": 30.0}      # publish rate of topics
        ],
        arguments=[
            '--config-filename', LaunchConfiguration('config_filename')
        ]
    )

    # Include rosbridge_server launch file
    rosbridge_launch_file = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])
    
    # Create the include launch description action
    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={
            'delay_between_messages': '0.0',
        }.items()
    )

    ld = LaunchDescription([
        config_file_arg,
        sim_bridge_node,
        rosbridge_server
    ])

    return ld