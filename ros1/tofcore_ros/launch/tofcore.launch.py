import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown, DeclareLaunchArgument, OpaqueFunction
import os


def launch_setup(context, *args, **kwargs):

    ts_camera = Node(
        package="tofcore",
        executable='tof_sensor',
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



    retval = [ ts_camera,  rviz, rqt_node]

    if LaunchConfiguration('with_ros1_bridge').perform(context).lower() == 'true':
        retval.append( Node(
            package='ros1_bridge', 
            executable='dynamic_bridge',
            arguments=['--bridge-all-2to1-topics'],
            output="screen",
            on_exit=Shutdown()
        ))


    return retval

def generate_launch_description():
    pkg_share = FindPackageShare(package='tofcore').find('tofcore')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz2/tofcore_basic_cloud.rviz')

    rvizconfig = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

  
    with_ros1_bridge = DeclareLaunchArgument(name='with_ros1_bridge', default_value='false',
                              description='Launch a ROS1 to ROS2 bridge with other nodes')



    return launch.LaunchDescription([
        rvizconfig,
        with_ros1_bridge,
        OpaqueFunction(function=launch_setup)
    ])