import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown, DeclareLaunchArgument, OpaqueFunction
import os
import yaml

# pkg_demo_share = launch_ros.substitutions.FindPackageShare(
#             package='tofcore_ros').find('tofcore_ros')

def launch_setup(context, *args, **kwargs):
    param_setting = launch.substitutions.LaunchConfiguration('config').perform(context)

    params_path = param_setting

    with open(params_path, "r") as fid:
        all_params = yaml.safe_load(fid)
     

     
    tof_params = all_params["/tof_sensor"]["ros__parameters"]
    ts_camera = Node(
        package="tof_sensor",
        executable='tof_sensor',
        name='tof_sensor',
        output='screen',
        parameters=[tof_params],
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
    

    retval = [  ts_camera,   rviz, rqt_node]

    if LaunchConfiguration('with_ros1_bridge').perform(context).lower() == 'true':
        retval.append( Node(
            package='ros1_bridge', 
            executable='dynamic_bridge',
            arguments=['--bridge-all-2to1-topics'],
            output="screen",
            on_exit=Shutdown()
        ))

    if LaunchConfiguration('with_ae').perform(context).lower() == 'true':

        
        ae_params = all_params["/automatic_exposure"]["ros__parameters"]
        retval.append(Node(
            package='controls_py',
            executable='automatic_exposure',
            name='automatic_exposure',
            output='screen',
            parameters=[ae_params],
            #condition=launch.conditions.IfCondition(LaunchConfiguration('auto-exposure')),
            on_exit=Shutdown()
        ))

    return retval

def generate_launch_description():
    pkg_share = FindPackageShare(package='tof_sensor').find('tof_sensor')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz2/tofcore_basic_cloud.rviz')
    defaul_config_path = os.path.join(pkg_share, 'config/config.yaml')

    rvizconfig = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')
    
    config = DeclareLaunchArgument(
        name='config',
        default_value=defaul_config_path,
        description='Absolute path to ae config file')
  
    with_ros1_bridge = DeclareLaunchArgument(name='with_ros1_bridge', default_value='false',
                              description='Launch a ROS1 to ROS2 bridge with other nodes')

    with_ae = DeclareLaunchArgument(name='with_ae', default_value='false',
                              description='Launch automatic exposure node')

    return launch.LaunchDescription([
        rvizconfig,
        with_ros1_bridge,
        config,
        with_ae,
        OpaqueFunction(function=launch_setup)
    ])