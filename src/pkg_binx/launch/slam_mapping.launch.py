import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    namePackage = 'pkg_binx'
    modelsPath   = os.path.join(get_package_share_directory(namePackage), 'models')
    pathModelFile = os.path.join(get_package_share_directory(namePackage), 'description', 'binx.urdf.xacro')
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), 'world', 'slam_world.sdf')
    bridge_params = os.path.join(get_package_share_directory(namePackage), 'parameters', 'bridge_parameters.yaml')
    slam_params   = os.path.join(get_package_share_directory(namePackage), 'config', 'slam_toolbox_params.yaml')

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazeboLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {pathWorldFile}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=modelsPath + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        gazeboLaunch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'binx', '-topic', 'robot_description'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True, 'use_lifecycle_manager': True}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox']
            }]
        ),
    ])
