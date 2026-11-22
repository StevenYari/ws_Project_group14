import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ackermann_description')
    xacro_file = os.path.join(pkg, 'urdf', 'ackermann_bot.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1) Gazebo con mundo vacio
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    # 2) Publica el modelo y los TF de los links
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}])

    # 3) Mete el robot en Gazebo
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description',
                            '-name', 'ackermann_bot', '-z', '0.05'])

    # 4) Bridge Gazebo <-> ROS 2
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/empty/model/ackermann_bot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/world/empty/model/ackermann_bot/joint_state', '/joint_states')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([gz_sim, rsp, spawn, bridge])
