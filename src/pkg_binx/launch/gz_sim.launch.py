# ------------------------------------------------------------------------------
# This file launches the gazebo simulation along with all the ros nodes required

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro



# *r e m o v e   l a t e r*
# ------------------------------------------------------------------------------------------------------------------
# !!!!!!!!!! IMPORTANT!!!!!!!!!!!
# paste this command in the terminal that you use to launch gazebo:
# export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/cloudy/Documents/Fluffy\ project/binx/src/pkg_binx/models
# (replace the path with your path)
# it's a bit scuffed but it's what works for now ToT
# ------------------------------------------------------------------------------------------------------------------







def generate_launch_description():

    robotXacroName = 'binx'

    namePackage = 'pkg_binx'

    modelFileRelativePath = 'description/binx.urdf.xacro'


    # HARDCODED PATH
    worldFileRelativePath = 'src/pkg_binx/world/Aworld.sdf'




    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)

    pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    print(pathWorldFile)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))




     # launch description

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments = {
            'gz_args': f'-r --verbose 4 {worldFileRelativePath}', 
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

    #launch descwiption
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)

    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)


    return launchDescriptionObject



