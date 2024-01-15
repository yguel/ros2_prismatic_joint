"""
Examples to run ros2 with this script:
  1. ros2 launch xy_unilivers_description display.launch.py
  2. ros2 launch xy_unilivers_description display.launch.py simu:=false
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

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'xy_unilivers_description'
    pkg_path = FindPackageShare(pkg_name)

    ##########
    ## URDF ##
    ##########
    
    model_path = PathJoinSubstitution([pkg_path,'urdf', 'xy_unilivers.urdf.xacro'])

    xacro_exe = PathJoinSubstitution([FindExecutable(name='xacro')])

    # Get URDF text data via xacro
    robot_description_content = Command([
        xacro_exe, ' ', model_path
    ])

    urdf_robot_desc = {'robot_description': robot_description_content}

    ################
    ## Simulation ##
    ################

    simu = LaunchConfiguration('simu')

    simu_arg = DeclareLaunchArgument(name='simu', 
        default_value='true', choices=['true', 'false'], 
        description="""Flag to enable simulation via gazebo. It will make the following actions:
         - use of simulated time
         - command of the joint via joint_state_publisher_gui
        """)
    
    # get_simu_mode = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([pkg_path, '/launch/display.launch.py']),
    #     launch_arguments={
    #         'simu': LaunchConfiguration('simu')}.items(),
    #         condition=IfCondition(simu)
    #                               )
    
    # Check if we're told to use sim time
    use_sim_time = { 'use_sim_time' : simu }

    ###########################
    ## Robot State Publisher ##
    ###########################

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[urdf_robot_desc, use_sim_time], # add other parameters here if required
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


    ###############################
    ## Joint State Publisher GUI ##
    ###############################

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[urdf_robot_desc, use_sim_time], # add other parameters here if required
        condition=IfCondition(simu)
    )

    ###########################
    ## Joint State Publisher ##
    ###########################

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[urdf_robot_desc, use_sim_time], # add other parameters here if required
        condition=UnlessCondition(simu)
    )


    #########################
    ## Collect all actions ##
    #########################
    actions = [
        simu_arg, # simulation argument
        robot_state_pub_node, # robot state publisher
        jsp_gui_node, # joint state publisher gui IF simu
        jsp_node, # joint state publisher UNLESS simu
        rviz_node, # rviz2
        ]

    #######################
    ## LaunchDescription ##
    #######################
    return LaunchDescription(actions)


    # rviz_config_path = PathJoinSubstitution([xy_unilivers_description_path, 'rviz', 'xy_unilivers.rviz'])

   
    # ld.add_action(simu_arg)

    # ld.add_action(IncludeLaunchDescription(
    #     # Here "display.launch.py" refers to the script of the urdf_launch package see https://github.com/ros/urdf_launch/blob/main/launch/display.launch.py
    #     PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
    #     launch_arguments={
    #         'urdf_package': 'xy_unilivers_description',
    #         'urdf_package_path': model_path,
    #         'rviz_config': rviz_config_path,
    #         'jsp_gui': LaunchConfiguration('simu')}.items()
    # ))

    # ld.add_action(IncludeLaunchDescription(
    #     PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
    #     launch_arguments={
    #         'use_sim_time': LaunchConfiguration('simu')}.items()
    # )

    # return ld
