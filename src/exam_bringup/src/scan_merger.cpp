// Forwards every robot's /robot_i/sensor_scan onto one topic so that a single
// octomap_server builds the team map. Each cloud keeps its own
// robot_i/sensor_at_scan frame, so octomap still ray-casts from the right origin.

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scan_merger");
  const int n_robots = node->declare_parameter<int>("n_robots", 3);

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/team/sensor_scan", 10);
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs;
  for (int i = 0; i < n_robots; i++) {
    subs.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/robot_" + std::to_string(i) + "/sensor_scan", 5,
      [pub](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { pub->publish(*msg); }));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
