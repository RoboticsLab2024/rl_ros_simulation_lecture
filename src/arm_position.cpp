#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("arm_test");

  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  RCLCPP_INFO(node->get_logger(), "node created");


  trajectory_msgs::msg::JointTrajectory command;

    command.joint_names = {"j0", "j1", "j2", "j3"};

    std::vector<std::string> joint_names = {"j0", "j1", "j2", "j3"};
    command.joint_names = joint_names;

    command.points.resize(1);
    command.points[0].positions.resize(joint_names.size());

    command.points[0].positions[0] = 0.2;
    command.points[0].positions[1] = 0.2;
    command.points[0].positions[2] = 0.2;
    command.points[0].positions[3] = 0.2;

    command.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap

    while (1) {
        publisher->publish(command);
        RCLCPP_INFO(node->get_logger(), "sending");
        std::this_thread::sleep_for(50ms);
        rclcpp::spin_some(node);
    }


  rclcpp::shutdown();

  return 0;
}
/*ros2 topic pub -r 10 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: ''
  },
  joint_names: ['j0', 'j1'],
  points: [{
    positions: [0.0, 0.2],
    velocities: [],
    time_from_start: { sec: 1, nanosec: 0 }
  }]
}"*/