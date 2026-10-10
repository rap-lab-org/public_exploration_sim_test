// Multi-robot exploration coordinator -- candidate template.
//
// The team is 3 identical wheeled robots, robot_0 .. robot_2. Use your RCH
// solver (include/explorer/rch.h) to decide which robot explores which
// region and in what order.
//
// Interface (all data in the global "map" frame):
//   subscribe  /projected_map             nav_msgs/OccupancyGrid  team map, 0.5 m cells
//              /robot_i/state_estimation  nav_msgs/Odometry       pose of robot i
//   publish    /robot_i/way_point         geometry_msgs/PointStamped
//
// The local planner only steers toward the way point inside its ~4 m sensing
// range. It does not plan around walls, so far-away way points will get the
// robot stuck. Feed it intermediate way points along a global path.
//
// Everything in this file may be rewritten; only the package name, the launch
// file name and the topic interface above are fixed.

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "explorer/rch.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Explorer : public rclcpp::Node
{
public:
  Explorer() : Node("explorer")
  {
    const int n_robots = declare_parameter<int>("n_robots", 3);

    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", 1,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_ = msg; });

    robots_.resize(n_robots);
    for (int i = 0; i < n_robots; i++) {
      const std::string ns = "/robot_" + std::to_string(i);
      auto & r = robots_[i];
      r.sub_odom = create_subscription<nav_msgs::msg::Odometry>(
        ns + "/state_estimation", 5,
        [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) { robots_[i].odom = msg; });
      r.pub_waypoint = create_publisher<geometry_msgs::msg::PointStamped>(ns + "/way_point", 5);
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { plan(); });
  }

private:
  struct Robot
  {
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_waypoint;
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
  };

  void plan()
  {
    // TODO:
    //  1. Extract exploration targets (e.g. frontier clusters) from map_.
    //  2. Build a rch::Problem: robot positions are the start nodes, targets
    //     the target nodes, costs are path lengths on the map (not straight lines).
    //  3. Call rch::solve(problem, w, time_limit) and give each robot the
    //     first target of its route.
    //  4. Send intermediate way points along a global path to that target.
    for (auto & r : robots_) {
      if (!r.odom) continue;
      geometry_msgs::msg::PointStamped wp;
      wp.header.frame_id = "map";
      wp.header.stamp = now();
      wp.point = r.odom->pose.pose.position;
      r.pub_waypoint->publish(wp);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<Robot> robots_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Explorer>());
  rclcpp::shutdown();
  return 0;
}
