import os,xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    this_pkg_path = os.path.join(get_package_share_directory('clear_space'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_model = 'satellite'

    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') #.urdf
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_desc}]
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

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/viewer.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher,
        joint_state_publisher_exec,
        open_rviz  # Ensure RViz is included in the launch process
    ])
