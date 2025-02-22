import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Set GAZEBO_PLUGIN_PATH to include your package's lib directory
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(get_package_share_directory('mobile_dd_robot'), 'lib')
    )

    # This name must match the robot name in your xacro file
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_dd_robot'
    modeFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/empty_world.world'
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modeFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # Process the xacro file to generate the robot description (URDF)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Include the Gazebo launch file from gazebo_ros
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items()
    )

    # Spawn the robot using the robot_description parameter (which includes the plugin details)
    spawnModelNode = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Create the launch description and add all actions
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(set_gazebo_plugin_path)
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    
    return launchDescriptionObject
