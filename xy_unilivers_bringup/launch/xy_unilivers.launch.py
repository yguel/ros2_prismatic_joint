# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ##########
    ## URDF ##
    ##########

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('xy_unilivers_description'), 'config', 'xy_unilivers.config.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    ##############################
    ## Robot Controller Manager ##
    ##############################
    
    # Define the path to the robot state controller configuration file
    controller_cfg = 'xy_unilivers_controllers.yaml'
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('xy_unilivers_description'),'config',controller_cfg,]
    )

    # Define controller_manager node parameters
    ctrl = {
        "position" : {
            "name" : "xy_unilivers_position_controller",
            "slider" : "xy_unilivers_slider_position_config.yaml",
            "args" : []#["-c", "/xy_unilivers/controller_manager"]
        },
        "trajectory" : {
            "name" : "xy_unilivers_trajectory_controller",
            "slider" : "xy_unilivers_slider_trajectory_config.yaml",
            "args" : []
        },
        "velocity" : {
            "name" : "xy_unilivers_joint_velocity_controller",
            "slider" : "xy_unilivers_slider_velocity_config.yaml",
            "args" : []
        }
    }
    ctrl_type = "velocity"

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[ctrl[ctrl_type]['name']]+ctrl[ctrl_type]['args'],
    )

    ###########################
    ## Robot State Publisher ##
    ###########################

    # Define robot state publisher node parameters
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )


    ##########
    ## RViz ##
    ##########

    # Define the path for rviz2 configuration file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('xy_unilivers_description'), 'rviz', 'xy_unilivers.rviz']
    )

    # Define rviz2 node parameters
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    #############################
    ## Joint State Broadcaster ##
    #############################

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']#,"--controller-manager", "/xy_unilivers/controller_manager"],
    )

    slider_config = PathJoinSubstitution(
        [
            FindPackageShare('xy_unilivers_bringup'),
            'config',
            ctrl[ctrl_type]['slider'],
        ]
    )

    slider_node = Node(
        package='slider_publisher', 
        executable='slider_publisher', 
        name='slider_publisher',
        parameters=[{'rate': 10.0}],
        arguments = [slider_config])

    #############################
    # Launch all the nodes
    #############################

    nodes = [
        rviz_node,
        controller_manager_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        slider_node,
    ]

    return LaunchDescription(nodes)