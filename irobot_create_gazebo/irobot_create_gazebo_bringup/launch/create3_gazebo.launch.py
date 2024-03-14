#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]

for pose_element in ['x1', 'y1', 'z1', 'yaw1', 'x2', 'y2', 'z2', 'yaw2']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')

    # Set ignition resource path
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(pkg_irobot_create_description).
                                                    parent.resolve())])

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri = SetEnvironmentVariable(name='GAZEBO_MODEL_URI', value=[''])

    # Paths
    create3_nodes_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'create3_nodes.launch.py'])
    dock_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'dock_description.launch.py'])
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'robot_description.launch.py'])
    rviz2_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'rviz2.launch.py'])
    gazebo_params_yaml_file = os.path.join(
        pkg_create3_gazebo_bringup, 'config', 'gazebo_params.yaml')

    # Launch configurations
    x1, y1, z1 = LaunchConfiguration('x1'), LaunchConfiguration('y1'), LaunchConfiguration('z1')
    yaw1 = LaunchConfiguration('yaw1')
    x2, y2, z2 = LaunchConfiguration('x2'), LaunchConfiguration('y2'), LaunchConfiguration('z2')
    yaw2 = LaunchConfiguration('yaw2')
    world_path = LaunchConfiguration('world_path')
    spawn_dock = LaunchConfiguration('spawn_dock')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path,
             'extra-gazebo-args', '--ros-args', '--params-file', gazebo_params_yaml_file],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gazebo_gui),
    )



    # Dock model and description
    # The robot starts docked. So we spawn the dock at the robot position.
    # We need to include an offset on the x and yaw to correctly "plug" the robot.

    # Dock 1
    x_dock1 = OffsetParser(x1, 0.157)
    yaw_dock1 = OffsetParser(yaw1, 3.1416)

    dock1_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch_file]),
        condition=IfCondition(spawn_dock),
        # The robot starts docked
        launch_arguments={'x': x_dock1, 'y': y1, 'z': z1, 'yaw': yaw_dock1, 'dock_id': 'dock1'}.items(),
    )

    spawn_dock1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        namespace='dock1',
        arguments=['-entity',
                   'standard_dock1',
                   '-topic',
                   'standard_dock_description',
                   '-x', x_dock1,
                   '-y', y1,
                   '-z', z1,
                   '-Y', yaw_dock1],
        output='screen',
        condition=IfCondition(spawn_dock),
    )

    # Dock 2
    x_dock2 = OffsetParser(x2, 0.157)
    yaw_dock2 = OffsetParser(yaw2, 3.1416)

    dock2_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch_file]),
        condition=IfCondition(spawn_dock),
        # The robot starts docked
        launch_arguments={'x': x_dock2, 'y': y2, 'z': z2, 'yaw': yaw_dock2, 'dock_id': 'dock2'}.items(),
    )

    spawn_dock2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        namespace='dock2',
        arguments=['-entity',
                   'standard_dock2',
                   '-topic',
                   'standard_dock_description',
                   '-x', x_dock2,
                   '-y', y2,
                   '-z', z2,
                   '-Y', yaw_dock2],
        output='screen',
        condition=IfCondition(spawn_dock),
    )

    # Create 3 robot models and descriptions

    # Robot 1
    robot1_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        launch_arguments={'robot_id': 'robot1'}.items(),
    )

    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        namespace='robot1',
        arguments=['-entity',
                   'robot1',
                   '-topic',
                   'robot_description',
                   '-x', x1,
                   '-y', y1,
                   '-z', z1,
                   '-Y', yaw1],
        output='screen',
    )

    create3_nodes_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch_file]),
        launch_arguments={'robot_id': 'robot1'}.items(),
    )

    # Robot 2
    robot2_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch_file]),
        launch_arguments={'robot_id': 'robot2'}.items(),
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        namespace='robot2',
        arguments=['-entity',
                   'robot2',
                   '-topic',
                   'robot_description',
                   '-x', x2,
                   '-y', y2,
                   '-z', z2,
                   '-Y', yaw2],
        output='screen',
    )

    create3_nodes_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch_file]),
        launch_arguments={'robot_id': 'robot2'}.items(),
    )

    # RVIZ2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz2_launch_file]),
        condition=IfCondition(use_rviz),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    # Gazebo processes
    ld.add_action(gz_resource_path)
    ld.add_action(gz_model_uri)
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Include robot1 description
    #ld.add_action(robot1_description)
    #ld.add_action(spawn_robot1)
    ld.add_action(spawn_dock1)
    ld.add_action(dock1_description)

    # Include robot1 description
    #ld.add_action(robot2_description)
    #ld.add_action(spawn_robot2)
    #ld.add_action(spawn_dock2)
    #ld.add_action(dock2_description)  

    # Include Create 3 nodes
    #ld.add_action(create3_nodes_robot1)
    #ld.add_action(create3_nodes_robot2)

    # Rviz
    ld.add_action(rviz2)

    return ld
