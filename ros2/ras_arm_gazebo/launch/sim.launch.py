"""The digital twin in Gazebo: workcell, arm with ros2_control (trajectory controller + gripper), cubes.

ros2 launch ras_arm_gazebo sim.launch.py                    # Gazebo window
ros2 launch ras_arm_gazebo sim.launch.py gui:=false rviz:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESC = get_package_share_directory('ras_arm_description')
SIM = get_package_share_directory('ras_arm_gazebo')
CONTROLLERS = os.path.join(SIM, 'config', 'controllers.yaml')
CUBE = 0.025          # m, CELL.cubeSize
COLOURS = {'RED': '0.85 0.2 0.2 1', 'BLUE': '0.16 0.44 0.86 1'}


def cube_sdf(name, colour, mass=0.012):
    i = mass * CUBE * CUBE / 6
    box = f'<geometry><box><size>{CUBE} {CUBE} {CUBE}</size></box></geometry>'
    return (f'<?xml version="1.0"?><sdf version="1.10"><model name="{name}"><link name="link">'
            f'<inertial><mass>{mass}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="c">{box}<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>'
            f'<visual name="v">{box}<material><ambient>{COLOURS[colour]}</ambient><diffuse>{COLOURS[colour]}</diffuse></material></visual>'
            f'</link></model></sdf>')


def gazebo(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    args = os.path.join(SIM, 'worlds', 'workcell.sdf') + ' -r -v 2' + ('' if gui else ' -s --headless-rendering')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    with open(os.path.join(DESC, 'config', 'arm.yaml')) as f:
        cell = yaml.safe_load(f)['cell']
    cubes = [('cube_red_cradle', 'RED', cell['cradle_x'], 0.0),     # where the firmware's RUN cycle picks
             ('cube_blue', 'BLUE', 0.16, -0.12), ('cube_red', 'RED', 0.15, 0.12)]
    xacro = lambda f, extra='': ParameterValue(Command(['xacro ', os.path.join(DESC, 'urdf', f), extra]), value_type=str)
    sim_time = {'use_sim_time': True}

    spawn_arm = Node(package='ros_gz_sim', executable='create', output='screen',
                     arguments=['-topic', 'robot_description', '-name', 'ras_arm'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                                  '--param-file', CONTROLLERS])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window (false: server only)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # package:// meshes resolve through the share directory
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(DESC)),
        OpaqueFunction(function=gazebo),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': xacro('ras_arm.urdf.xacro', f' sim:=true controllers:={CONTROLLERS}')}, sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='cell_state_publisher',
             parameters=[{'robot_description': xacro('ras_cell.urdf.xacro')}, sim_time],
             remappings=[('robot_description', 'cell_description')]),
        Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'cell_description', '-name', 'ras_cell']),
        spawn_arm,
        *[Node(package='ros_gz_sim', executable='create',
               arguments=['-string', cube_sdf(name, colour), '-name', name, '-x', str(x), '-y', str(y), '-z', str(CUBE / 2 + 0.001)])
          for name, colour, x, y in cubes],
        RegisterEventHandler(OnProcessExit(target_action=spawn_arm, on_exit=[controllers])),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(DESC, 'rviz', 'ras_arm.rviz')], parameters=[sim_time],
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),                # RViz's GLX window needs X11
    ])
