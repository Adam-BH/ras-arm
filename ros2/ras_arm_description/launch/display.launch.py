"""RViz: the arm (sliders for every joint) and the workcell. No simulation.

ros2 launch ras_arm_description display.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('ras_arm_description')
    xacro = lambda f: ParameterValue(Command(['xacro ', PathJoinSubstitution([share, 'urdf', f])]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': xacro('ras_arm.urdf.xacro')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='cell_state_publisher',
             parameters=[{'robot_description': xacro('ras_cell.urdf.xacro')}],
             remappings=[('robot_description', 'cell_description')]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        # xcb: RViz's GLX render window needs X11 (Wayland desktops and WSLg: "Invalid parentWindowHandle")
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution([share, 'rviz', 'ras_arm.rviz'])],
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),
    ])
