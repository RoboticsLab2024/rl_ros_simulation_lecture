from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from launch.conditions import IfCondition, UnlessCondition


iiwa = False

def to_bool(value):
        return value.lower() in ["true", "1", "yes", "y", "vero", "v"]

def generate_launch_description():
    declared_arguments = []

    arm_description_path = os.path.join(
        get_package_share_directory('arm_description'))

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "standing.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'config',
            default_value= os.path.join(
                arm_description_path,
                'config',
                'pos_controller.yaml'
            ),
            description='YAML configuration file'
        ),
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'anymal',
            default_value='false',
            description='YAML configuration file'
        ),
    )
  
    urdf_arm = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")
    urdf_anymal = os.path.join(arm_description_path, "urdf", "anymal.urdf.xacro")
    urdf_iiwa = os.path.join(arm_description_path, "urdf", "iiwa.urdf.xacro")
    rviz_config = os.path.join(arm_description_path, "config", "rviz", "standing.rviz")

    """with open(urdf_arm, 'r') as infp:
        arm_desc = infp.read()

    with open(urdf_anymal, 'r') as infp:
        anymal_desc = infp.read()  #If you use the xacro command you don't need to read the file anymore"""

    robot_description_arm = {"robot_description": Command(['xacro ', urdf_arm, " pos_j2:=", "1.75"])}  
    robot_description_anymal = {"robot_description": Command(['xacro ', urdf_anymal])}
    robot_description_iiwa = {"robot_description": Command(['xacro ', urdf_iiwa])}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    if(iiwa):
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description_iiwa,
                        {"use_sim_time": True},
                ],
            condition=UnlessCondition(LaunchConfiguration('anymal')),
            remappings=[('/robot_description', '/robot_description')]
        )
    else:
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description_arm,
                        {"use_sim_time": True},
                ],
            condition=UnlessCondition(LaunchConfiguration('anymal')),
            remappings=[('/robot_description', '/robot_description')]
        )

    robot_state_publisher_node_anymal = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_anymal,
                    {"use_sim_time": True},
            ],
        condition=IfCondition(LaunchConfiguration('anymal')),
        remappings=[('/robot_description', '/robot_description')]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
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

    position = [0.0, 0.0, 0.65]

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

    #GAZEBO CLASSIC
    gazebo_classic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={"gui": "True", 'paused': 'False'}.items(),
    )
    position = [0.0, 0.0, 0.65]
    # Spawn robot. Each argument passed must be a string
    gazeboClassic_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=["-entity", "monopod", #arguments must always be string because strings are the argument passed by cli
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),
                    '-topic', '/robot_description'],
        output="screen", #-file needs the path, not the string
    )

    classic = [gazebo_classic, gazeboClassic_spawn_robot]

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    ) #among the controller of controller_manager you are saying to launchg the joint_state_broadcaster

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],  
    ) 

    #Launch the ros2 controllers after the model spawns in Gazebo 
    delay_joint_traj_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_trajectory_controller],
        )
    )

    delay_joint_state_broadcaster = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        )
    )
    
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )
 

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,  
        robot_state_publisher_node_anymal,
        *ign,
        bridge_camera,
        delay_joint_traj_controller, 
        delay_joint_state_broadcaster
    ]
    
    """In rviz, quando selezioni il modello, devi mettere il topic /robot_descriptio in Description topic"""

    return LaunchDescription(declared_arguments + nodes_to_start) 
