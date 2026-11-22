"""Launch unico: Gazebo + robot + bridge (sim.launch.py) + controlador Pure Pursuit.

Uso:
    ros2 launch ackermann_description full_sim.launch.py
    ros2 launch ackermann_description full_sim.launch.py lookahead:=0.8 speed:=0.3
    ros2 launch ackermann_description full_sim.launch.py path_type:=circle radius:=1.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ackermann_description')

    args = [
        DeclareLaunchArgument('path_type', default_value='lemniscate',
                              description='lemniscate | circle | oval'),
        DeclareLaunchArgument('lookahead', default_value='0.6', description='Ld [m]'),
        DeclareLaunchArgument('speed', default_value='0.4', description='velocidad lineal [m/s]'),
        DeclareLaunchArgument('size', default_value='3.0',
                              description='infinity: semi-largo a [m] (radio minimo = a/3)'),
        DeclareLaunchArgument('radius', default_value='1.5', description='circle / oval [m]'),
        DeclareLaunchArgument('straight', default_value='2.0', description='oval: largo de rectas [m]'),
        DeclareLaunchArgument('delay', default_value='10.0',
                              description='segundos de espera antes de arrancar el controlador'),
    ]

    # Simulacion: Gazebo + robot + bridge
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')))

    # Controlador (arranca despues de 'delay' s para que el robot ya este en el mundo)
    controller = Node(
        package='ackermann_control',
        executable='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'path_type': ParameterValue(LaunchConfiguration('path_type'), value_type=str),
            'lookahead': ParameterValue(LaunchConfiguration('lookahead'), value_type=float),
            'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            'size': ParameterValue(LaunchConfiguration('size'), value_type=float),
            'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
            'straight': ParameterValue(LaunchConfiguration('straight'), value_type=float),
        }])

    delayed = TimerAction(
        period=LaunchConfiguration('delay'),
        actions=[controller])

    return LaunchDescription(args + [sim, delayed])
