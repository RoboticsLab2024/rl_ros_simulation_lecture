# ROS 2 SIMULATION 16/10/2024

## Overview
Brief description of your project.
```bash
git clone 
```

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
ros2 launch links_urdf
```
#![Example Image](images/rviz_1.png)
<img src="images/rviz_1.png" alt="Description of the image" width="250"/>
If you have launched the joint-state-publisher-gui, a slider bar will appear to move joints. (This package might need to be installed) 

On another terminal open rqt. You can check both the tf_tree published by the robot_state_publisher 

<img src="images/tf_tree.png" alt="Description of the image" width="150"/>

and the ros2 computational graph:
<img src="images/graph.png" alt="Description of the image" width="150"/>
