cmake_minimum_required(VERSION 3.8)
project(links_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)

#ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)
 
ament_package()



#ros_gz_sim github
