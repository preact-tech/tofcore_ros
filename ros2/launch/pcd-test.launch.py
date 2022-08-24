import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown, DeclareLaunchArgument
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='t10').find('t10')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz2/preact_basic_cloud.rviz')

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name='transformer',
        arguments="0 0 0 0 0 0 unknown t21_frame".split()
    )

    ts_camera = Node(
        package="t10",
        executable='t10_sensor',
        namespace='truesense',
        output='screen',
        parameters=[{'stream_type': "distance_amplitude"}],
        on_exit=Shutdown()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        on_exit=Shutdown()
    )
    
    rqt_node= Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="Configure",
        arguments=['--force-discover'],
        output="screen",
        on_exit=Shutdown()
    )

    rvizconfig = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    return launch.LaunchDescription([
        rvizconfig,
        static_transform,
        ts_camera,
        rviz,
        rqt_node
    ])
