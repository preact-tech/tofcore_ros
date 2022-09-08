import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown, DeclareLaunchArgument
import os
from datetime import datetime

def generate_launch_description():

    ts_camera = Node(
        package="t10",
        executable='t10_sensor',
        output='screen',
        parameters=[{
        'stream_type': "distance_amplitude",
        'integration_time0' : LaunchConfiguration('int_time0'),
        'integration_time1' : LaunchConfiguration('int_time1'),
        'integration_time2' : LaunchConfiguration('int_time2'),
        'hdr_mode' : LaunchConfiguration('hdr_mode'),
        }],
        on_exit=Shutdown()
    )

    record_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all', '-o', LaunchConfiguration('bagfile') ], 
        output='screen',
        on_exit=Shutdown()
    )

    int_time0 = DeclareLaunchArgument(
        'int_time0',
        default_value='50',
        description='integration time 0')

    int_time1 = DeclareLaunchArgument(
        'int_time1',
        default_value='250',
        description='integration time 1 (for HDR)')

    int_time2 = DeclareLaunchArgument(
        'int_time2',
        default_value='700',
        description='integration time 2 (for HDR)')

    hdr_mode = DeclareLaunchArgument(
        'hdr_mode',
        default_value='o',
        description='HDR mode (off, temporal, spacetial')

    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bagfile_arg = DeclareLaunchArgument(
        'bagfile',
        description='Path to location to store bagfile',
        default_value="t10_{}".format(timestamp))


    return launch.LaunchDescription([
        int_time0,
        int_time1,
        int_time2,
        hdr_mode,
        bagfile_arg,
        ts_camera,
        record_process,
    ])
