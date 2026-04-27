# ------------------------------------------------------------------------------
# This file launches the gazebo simulation along with all the ros nodes required

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
import xacro



def generate_launch_description():

    robotXacroName = 'binx'
    namePackage = 'pkg_binx'
    modelFileRelativePath = 'description/binx.urdf.xacro'
    modelsPath = os.path.join(get_package_share_directory(namePackage),'models')
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage),'world','slam_world.sdf')
    print(pathWorldFile)

    nav2_params = os.path.join(get_package_share_directory(namePackage),                                 
      'config',                                                                 
      'nav2_params.yaml'
    )                                                                             
            
    map_yaml = os.path.join(get_package_share_directory(namePackage),
      'maps',                                                                   
      'maze_map.yaml'
    )

    robotDescription = xacro.process_file(pathModelFile).toxml()
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))

     # launch description
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments = {
            'gz_args': f'-r --verbose 4 {pathWorldFile}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # robot state publisher node
    nodeRobotStatePublisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{
            'robot_description' : robotDescription,
            'use_sim_time': True
        }]
    )


    # gazebo node
    spawnModelNodeGazebo = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        arguments = [
                '-name', robotXacroName,
                '-topic', 'robot_description',
        ],
        output = 'screen'
    )


    # main bridge
    # -----------
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = [
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output = 'screen',
    )

    # Dedicated image bridge for better performance
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'],
        output='screen',
    )
    
    # EKF node
    ekf_params = os.path.join(
        get_package_share_directory(namePackage),
        'config',
        'ekf_params.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}]
    )

    # slam toolbox
    slam_params = os.path.join(
        get_package_share_directory('pkg_binx'),
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}, {'use_lifecycle_manager': True}]
    )

    #nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 30.0,
            'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']
        }]
    )

    #mission
    spawn_trash = TimerAction(
        period=8.0, # only after 8s, to give binx time to spawn and start up
        actions=[ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory(namePackage), 'scripts', 'spawn_trash.py')],
            output='screen'
        )]
    )

    lifecycle_manager_delayed = TimerAction(
        period=10.0, #after 10s, to give binx time and for slam toolbox to start up (since nav2 depends on it)
        actions=[lifecycle_manager]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
            'yaml_filename': map_yaml
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
        }]
    )       

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
        }]
    )       

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
        }]
    )   

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True,
        }],
        arguments=['--ros-args', '--log-level', 'bt_navigator:=WARN']
    )

    #launch descwiption
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=modelsPath + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')))
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(start_gazebo_ros_image_bridge_cmd)
    launchDescriptionObject.add_action(spawn_trash)
    launchDescriptionObject.add_action(lifecycle_manager_delayed)
    launchDescriptionObject.add_action(map_server_node)
    launchDescriptionObject.add_action(amcl_node)
    launchDescriptionObject.add_action(planner_server_node)
    launchDescriptionObject.add_action(controller_server_node)
    launchDescriptionObject.add_action(bt_navigator_node)

    return launchDescriptionObject



