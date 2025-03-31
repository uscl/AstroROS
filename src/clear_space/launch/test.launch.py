import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    this_pkg_path = os.path.join(
        get_package_share_directory('clear_space'))
    # Launch arguments
    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    robot_model = 'satellite'
    world_file = "no_gravity.sdf"
    # Set ign sim resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/ign_viewer.rviz")],
    )
    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r'])

        ]
    )
    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') #.urdf
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'satellite',
                   '-allow_renaming', 'false',
                   '-x', '0', '-y', '0', '-z', '0'],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_desc}]
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/bus_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/bus_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/bus_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/world_model/model/'+robot_model+'/joint_state@sensor_msgs/msg/JointState',
        ],
        output='screen',
        remappings=[
            ('/world/world_model/model/'+robot_model+'/joint_state', '/'+robot_model+'/joint_states')
        ]
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
    # load_imu_sensor_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'imu_sensor_broadcaster'],
    #     output='screen'
    # )
    plot_node = Node(
                package="orbit_viewer",
                executable="orbit_viewer",
                name="orbit_plot_node",
    )
    return LaunchDescription(
        [
            simu_time,
            ign_resource_path,
            # open_rviz,
            open_ign,
            ignition_spawn_entity,
            robot_state_publisher,
            load_joint_state_broadcaster,
            load_arm_controller,
            load_rw_controller,
            bridge,
            plot_node
        ]
    )
