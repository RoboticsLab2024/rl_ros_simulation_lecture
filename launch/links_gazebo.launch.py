from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
 

def generate_launch_description():

    declared_arguments = [] 
 
    link_description_path = get_package_share_directory('links_urdf')

    urdf_example_lesson = os.path.join(link_description_path, "urdf", "links.urdf")
    urdf_example_lesson_xacro = os.path.join(link_description_path, "urdf", "links.urdf.xacro")


    with open(urdf_example_lesson, 'r') as infp:
        link_desc = infp.read()

    robot_description_links = {"robot_description": link_desc}

    r_d_x = {"robot_description":Command(['xacro ', urdf_example_lesson_xacro])}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )  

    robot_state_publisher_node_links = Node(
        package="robot_state_publisher", #ros2 run robot_state_publisher robot_state_publisher
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_links,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )

    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    position = [0.0, 0.0, 0.45]

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'arm',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),],
    )


    ign = [gazebo_ignition, gz_spawn_entity]

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node_links,
        *ign
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start) 

