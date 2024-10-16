# ROS 2 SIMULATION 16/10/2024

## USAGE
Add within your cmake:
```
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)
```

In order to make the launch file visible to your ROS system.

Build the package with
```
colcon build --packages-select links_urdf
```

Launch robot-state-publisher, joint-state-publisher and rviz2 with:
```
ros2 launch links_urdf links.launch.py
```

<img src="images/rviz_1.png" alt="Description of the image" width="400"/>
If you have launched the joint-state-publisher-gui, a slider bar will appear to move the joints. (This package may need to be installed)

In another terminal, open rqt. You can check the TF tree published by the robot_state_publisher.

<img src="images/tf_tree.png" alt="Description of the image" width="450"/>

and the ros2 computational graph:

<img src="images/graph.png" alt="Description of the image" width="550"/>

From there, you can see how the robot_state_publisher is publishing both the transformations and the robot_description



Spawn the robot in Gazebo using the *create* node:
```
ros2 launch links_urdf links_gazebo.launch.py
```

# Exercise

* Download the iiwa_description package from [iiwa repository](https://github.com/ICube-Robotics/iiwa_ros2/tree/main)
* Create a launch file to spawn the Iiwa in Gazebo (you just need to change the paths inside the launch file provided in this repo) 
* Check if the robot spawns both in Gazebo and Rviz





