#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Samuel Bamert
#
# Launch Create(R) 3 nodes

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution

ARGUMENTS = []

# Set the robot and dock pose
for pose_element, default_value in zip(['x1', 'y1', 'yaw1', 'x2', 'y2', 'yaw2'], ['2.0', '0.0', '0.0', '-2.0', '0.0', '0.0']):
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value=default_value,
                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():
	#Directories
    irobot_create_gazebo_bringup_dir = get_package_share_directory('irobot_create_gazebo_bringup')

    # Paths
    create3_launch_file = PathJoinSubstitution(
        [irobot_create_gazebo_bringup_dir, 'launch', 'create3_gazebo.launch.py'])

    # Includes
    world_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_launch_file])
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    #Actions
    ld.add_action(world_spawn)	

    return ld