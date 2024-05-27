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

    # Specify the name of the robot description package
    pkg_name = 'xy_unilivers_description'
    pkg_path = FindPackageShare(pkg_name)

    ##########
    ## URDF ##
    ##########

    # Specify the path to the xacro file within the description package
    model_path = PathJoinSubstitution([pkg_path,'config', 'xy_unilivers_secured.config.xacro'])

    xacro_exe = PathJoinSubstitution([FindExecutable(name='xacro')])

    # Get URDF text data via xacro
    robot_description_content = Command([
        xacro_exe, ' ', model_path
    ])

    ###########################
    ## Robot State Publisher ##
    ###########################

    rsp_params = {
        'robot_description': robot_description_content,
        'use_sim_time': False
    }

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[rsp_params], # add other parameters here if required
    )

    #############################
    ## Joint State Broadcaster ##
    #############################

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    ######################
    ## Robot Controller ##
    ######################

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xy_unilivers_secured_1d_velocity_controller'],
    )

    # Define the path to the robot state controller configuration file
    controller_cfg = 'xy_unilivers_secured_controllers.yaml'
    robot_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare('xy_unilivers_description'),'config',controller_cfg,]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[rsp_params, robot_controllers_cfg],
        output='both',
    )

    ###########
    ## RVIZ2 ##
    ###########

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

    #######################
    ## Command interface ##
    #######################
    cmd_slider_cfg = PathJoinSubstitution(
        [
            FindPackageShare('xy_unilivers_bringup'),
            'config',
            "xy_unilivers_linear_secured_velocity_rpm_joint_config.yaml",
        ]
    )

    cmd_slider_node = Node(
        package='slider_publisher', 
        executable='slider_publisher', 
        name='slider_publisher',
        parameters=[{'rate': 100.0}],
        arguments = [cmd_slider_cfg])


    #########################
    ## Collect all actions ##
    #########################
    actions = [
        robot_state_pub_node, # robot state publisher
        controller_manager_node, # controller manager
        cmd_slider_node, # slider publisher
        rviz_node, # rviz2
        joint_state_broadcaster, # joint state broadcaster
        robot_controller_spawner,
        ]

    #######################
    ## LaunchDescription ##
    #######################
    return LaunchDescription(actions)