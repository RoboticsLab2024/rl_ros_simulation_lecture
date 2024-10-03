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

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "standing.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
 
    urdf_path = os.path.join(
        get_package_share_directory('arm_description'), "urdf", "arm.urdf"
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    description_file = LaunchConfiguration("description_file") #this is the path
 

    rviz_config = os.path.join(
        get_package_share_directory('arm_description'), "config", "rviz", "standing.rviz"
    )

    robot_description = {"robot_description": robot_desc}  

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
                {"use_sim_time": True}
            ],
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

    position = [0.0, 0.0, 0.5]

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'monopod',
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


    
    robot_controllers = PathJoinSubstitution([FindPackageShare("arm_description"), "config", "pos_controller.yaml",])

    #Controller manager load the controllers defined in the yaml file
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node", 
        name='controller_manager',
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    ) #among the controller of controller_manager you are saying to launchg the joint_state_broadcaster

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "20"],  
    ) 

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
        robot_state_publisher_node, #publish pose of each link and transformation to know ehre the robot is
        rviz_node,
        *ign,
        bridge_camera
        #control_node, #if your hardware interface is the simulator, the ros-control will be the one specified in the plugin gazebo, 
        #joint_trajectory_controller,
        #joint_state_broadcaster, #publish the state of the robot as sensor-masgs for control (like the joints pos e vel)
        #delay_joint_traj_controller, #ros controller chosen. I could also use a psition controller
        #delay_joint_state_broadcaster
    ]
    
    """In rviz, quando selezioni il modello, devi mettere il topic /robot_descriptio in Description topic"""

    return LaunchDescription(declared_arguments + nodes_to_start) #its argument must to be a list
