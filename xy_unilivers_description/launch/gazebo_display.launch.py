"""
Examples to run ros2 with this script:
  1. ros2 launch xy_unilivers_description gazebo_display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Specify the name of the robot description package
    pkg_name = 'xy_unilivers_description'
    pkg_path = FindPackageShare(pkg_name)

    ##########
    ## URDF ##
    ##########
    
    # Specify the path to the xacro file within the description package
    model_path = PathJoinSubstitution([pkg_path,'urdf', 'xy_unilivers.urdf.xacro'])

    xacro_exe = PathJoinSubstitution([FindExecutable(name='xacro')])

    # Get URDF text data via xacro
    robot_description_content = Command([
        xacro_exe, ' ', model_path
    ])

    ################
    ## Simulation ##
    ################
    

    ###########################
    ## Robot State Publisher ##
    ###########################

    rsp_params = {
        'robot_description': robot_description_content,
        'use_sim_time': True
    }

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[rsp_params], # add other parameters here if required
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
    ## Gazebo Simulation ##
    #######################
    # Include the Gazebo launch file, provided by the gazebo_ros package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_launch_file])
                )
    

 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros',
                         executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'xy_unilivers'],
                        output='screen'
    )

    #######################
    ## Command interface ##
    #######################
    cmd_slider_cfg = PathJoinSubstitution(
        [
            FindPackageShare('xy_unilivers_bringup'),
            'config',
            "linear_velocity_slider.yaml",
        ]
    )

    cmd_slider_node = Node(
        package='slider_publisher', 
        executable='slider_publisher', 
        name='slider_publisher',
        parameters=[{'rate': 10.0}],
        arguments = [cmd_slider_cfg])


    #########################
    ## Collect all actions ##
    #########################
    actions = [
        gazebo, # gazebo simulation
        robot_state_pub_node, # robot state publisher
        cmd_slider_node, # slider publisher
        rviz_node, # rviz2
        spawn_entity # spawn entity
        ]

    #######################
    ## LaunchDescription ##
    #######################
    return LaunchDescription(actions)