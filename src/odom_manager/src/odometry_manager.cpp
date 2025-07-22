#include "odometry_manager.h"

OdometryManager::OdometryManager() : rate(100.0) {
  double agv1_x, agv1_y, agv1_z, agv1_yaw;
  double agv2_x, agv2_y, agv2_z, agv2_yaw;
  double uav1_x, uav1_y, uav1_z, uav1_yaw;
  double uav2_x, uav2_y, uav2_z, uav2_yaw;

  ros::param::get("agv1/x_init", agv1_x);
  ros::param::get("agv1/y_init", agv1_y);
  ros::param::get("agv1/z_init", agv1_z);
  ros::param::get("agv1/theta_init", agv1_yaw);

  ros::param::get("agv2/x_init", agv2_x);
  ros::param::get("agv2/y_init", agv2_y);
  ros::param::get("agv2/z_init", agv2_z);
  ros::param::get("agv2/theta_init", agv2_yaw);

  ros::param::get("uav1/x_init", uav1_x);
  ros::param::get("uav1/y_init", uav1_y);
  ros::param::get("uav1/z_init", uav1_z);
  ros::param::get("uav1/theta_init", uav1_yaw);

  ros::param::get("uav2/x_init", uav2_x);
  ros::param::get("uav2/y_init", uav2_y);
  ros::param::get("uav2/z_init", uav2_z);
  ros::param::get("uav2/theta_init", uav2_yaw);

  // 初始化 map
  vehicle_definitions = {
      {"agv1", InitialStatus{agv1_x, agv1_y, agv1_z, agv1_yaw, VehicleType::AGV,
                             true, "package://odom_manager/model/agv.dae"}},
  };

  for (const auto &[name, status] : vehicle_definitions) {
    if (status.type == VehicleType::AGV) {
      vehicles.emplace(
          name, std::make_shared<AGV>(name, nh, status.x_init, status.y_init,
                                      status.theta_init, status.use_model,
                                      status.mesh_resource));
    } else {
      vehicles.emplace(
          name, std::make_shared<UAV>(name, nh, status.x_init, status.y_init,
                                      status.z_init, status.theta_init,
                                      status.use_model, status.mesh_resource));
    }

    vehicles[name]->cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "/" + name + "/cmd_vel", 10,
        std::bind(&OdometryManager::cmdCallback, this, std::placeholders::_1,
                  name));
  }
}

void OdometryManager::run() {
  tf2_ros::TransformBroadcaster broadcaster;
  double prev_time = ros::Time::now().toSec();
  while (ros::ok()) {
    double current_time_sec = ros::Time::now().toSec();
    double dt = current_time_sec - prev_time;
    prev_time = current_time_sec;
    ros::Time current_time = ros::Time::now();
    for (auto &pair : vehicles) {
      pair.second->updatePose(dt);
      pair.second->publishData(current_time, broadcaster);
    }
    ros::spinOnce();
    rate.sleep();
  }
}

void OdometryManager::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg,
                                  const std::string &vehicle_name) {
  auto it = vehicles.find(vehicle_name);
  if (it != vehicles.end()) {
    it->second->vx = msg->linear.x;
    it->second->vy = msg->linear.y;
    it->second->vz = msg->linear.z;
    it->second->angular_velocity = msg->angular.z;
  } else {
    ROS_WARN("[Odom Manager]received cmd_vel for unknown vehicle: %s",
             vehicle_name.c_str());
  }
}