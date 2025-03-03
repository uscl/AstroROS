import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo,IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessStart
from launch.descriptions import Executable

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('satellite_description')
    default_model_path = os.path.join(pkg_share, 'xacro/satellite.xacro')
    default_model_path_1 = os.path.join(pkg_share, 'xacro/satellite.xacro.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz/viewer.rviz')
    world_name = 'no_gravity.world.sdf'

    xacro_file = os.path.join(pkg_share,
                              'xacro',
                              'satellite.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml()

    sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )

    open_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('ign_args', [pkg_share+"/worlds/"+world_name, ' -v 4', ' -r'])

        ]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', default_model_path])}]
        # parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['urdf', default_model_path_1])}]
    )

    joint_state_publisher_exec = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run ',
            ' joint_state_publisher_gui ',
            ' joint_state_publisher_gui'
        ]],
        shell=True
    )

    robot_state = RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[
                    LogInfo(msg='robot_state_publisher started, starting joint state publisher'),
                    # joint_state_publisher_exec
                ]
            )
        )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'satellite',
                   '-allow_renaming', 'true',
                   '-x', '0', '-y', '0', '-z', '1'],
    )

    # ros_ign_bridge = ExecuteProcess(
    # cmd=[
    #     'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
    #     '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
    #     '--ros-args', '--remap', '/satellite/tf:=/tf'
    # ],
    # output='screen'
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        sim_time,

        robot_state_publisher_node,
        robot_state,
        rviz_node,

        open_world,
        ignition_spawn_entity,
        # ros_ign_bridge
    ])
