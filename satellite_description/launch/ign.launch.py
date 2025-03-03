# Copyright 2022 Open Source Robotics Foundation, Inc.
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
#
# Author: Denis Stogl (Stogl Robotics Consulting)
#

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world_file = 'no_gravity.world.sdf' # warehouse
    satellite_ros2_control = os.path.join(
        get_package_share_directory('satellite_description'))

    xacro_file = os.path.join(satellite_ros2_control,
                              'xacro',
                              'satellite.xacro')
    default_model_path = os.path.join(satellite_ros2_control, 'xacro/satellite.xacro')
    doc = xacro.parse(open(xacro_file))
    # print("====================================================================================")
    # print(doc)
    # print("====================================================================================")
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', default_model_path])}]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'satellite',
                   '-allow_renaming', 'true',
                   '-x', '0', '-y', '0', '-z', '1'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )
    load_rw_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'rw_controller'],
        output='screen'
    )
    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster'],
        output='screen'
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/bus_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/bus_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/bus_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        # parameters=[{'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'}],
        output='screen',
        # remappings=[            # ign topic -l
        #     ('/model/'+robot_ns+'/cmd_vel', '/'+robot_ns+'/cmd_vel'),
        #     ('/model/'+robot_ns+'/odometry', '/'+robot_ns+'/odom'),
        #     ('/world/world_model/model/'+robot_ns+'/joint_state', '/'+robot_ns+'/joint_states'),

        #     # ('/'+robot_ns+'/imu', '/'+robot_ns+'/imu'),
        #     ('/'+robot_ns+'/lidar', '/'+robot_ns+'/scan'),
        #     ('/model/'+robot_ns+'/battery/linear_battery/state', '/'+robot_ns+'/battery/state'),
        # ]
    )
    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('ign_args', [satellite_ros2_control+"/worlds/"+world_file,' -r'])]),
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_rw_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rw_controller,
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),
        # node_robot_state_publisher,
        ignition_spawn_entity,
        bridge,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
