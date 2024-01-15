"""
Examples to run ros2 with this script:
  1. ros2 launch xy_unilivers_description rviz_display.launch.py
  2. ros2 launch xy_unilivers_description rviz_display.launch.py model:=urdf/xy_unilivers_simplified.urdf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('xy_unilivers_description')
    default_model_path = PathJoinSubstitution([pkg_path,'urdf', 'xy_unilivers.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([pkg_path, 'rviz', 'xy_unilivers.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to xy_unilivers package'))

    ld.add_action(IncludeLaunchDescription(
        # Here "display.launch.py" refers to the script of the urdf_launch package see https://github.com/ros/urdf_launch/blob/main/launch/display.launch.py
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'xy_unilivers_description',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    return ld
