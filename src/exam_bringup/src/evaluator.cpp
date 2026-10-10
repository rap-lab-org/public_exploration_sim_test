// Team-level exploration evaluator.
//
// Merges the registered scans of all robots into one voxel set and reports the
// explored volume of the whole team. Voxel size and counting rule match
// visualization_tools (count of occupied 0.5 m voxels), so per-robot and team
// numbers are directly comparable.
//
// The clock starts on the first /robot_i/way_point message from any robot and
// stops at time_limit. Results are written to <log_dir>/<stamp>/.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace fs = std::filesystem;

class Evaluator : public rclcpp::Node
{
public:
  Evaluator() : Node("exam_evaluator")
  {
    n_robots_ = declare_parameter<int>("n_robots", 3);
    time_limit_ = declare_parameter<double>("time_limit", 300.0);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.5);
    log_dir_ = declare_parameter<std::string>("log_dir", "exam_results");

    robots_.resize(n_robots_);
    for (int i = 0; i < n_robots_; i++) {
      const std::string ns = "/robot_" + std::to_string(i);
      auto & r = robots_[i];
      r.sub_scan = create_subscription<sensor_msgs::msg::PointCloud2>(
        ns + "/registered_scan", 5,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { scanHandler(msg); });
      r.sub_odom = create_subscription<nav_msgs::msg::Odometry>(
        ns + "/state_estimation", 5,
        [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomHandler(i, msg); });
      r.sub_goal = create_subscription<geometry_msgs::msg::PointStamped>(
        ns + "/way_point", 5,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr) { start(); });
    }

    auto latched = rclcpp::QoS(1).transient_local();
    pub_volume_ = create_publisher<std_msgs::msg::Float32>("/exam/explored_volume", 5);
    pub_time_ = create_publisher<std_msgs::msg::Float32>("/exam/elapsed_time", 5);
    pub_finished_ = create_publisher<std_msgs::msg::Bool>("/exam/finished", latched);
    pub_voxels_ = create_publisher<sensor_msgs::msg::PointCloud2>("/exam/explored_voxels", 2);

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "Evaluating %d robots, time limit %.0f s. "
                "Clock starts on the first way_point.", n_robots_, time_limit_);
  }

private:
  struct Robot
  {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal;
    bool has_pose = false;
    double x = 0, y = 0, z = 0;
    double distance = 0;
  };

  enum class State { Waiting, Running, Finished };

  int64_t voxelKey(float x, float y, float z) const
  {
    // 21 bits per axis, offset so negative indices stay positive.
    const int64_t off = 1 << 20;
    const int64_t ix = static_cast<int64_t>(std::floor(x / voxel_size_)) + off;
    const int64_t iy = static_cast<int64_t>(std::floor(y / voxel_size_)) + off;
    const int64_t iz = static_cast<int64_t>(std::floor(z / voxel_size_)) + off;
    return (ix << 42) | (iy << 21) | iz;
  }

  double elapsed() const { return (now() - start_time_).seconds(); }

  void start()
  {
    if (state_ != State::Waiting) return;
    state_ = State::Running;
    start_time_ = now();

    char stamp[32];
    std::time_t t = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    run_dir_ = fs::path(log_dir_) / stamp;
    fs::create_directories(run_dir_);
    csv_ = std::fopen((run_dir_ / "metrics.csv").c_str(), "w");
    std::fprintf(csv_, "time,explored_volume");
    for (int i = 0; i < n_robots_; i++) std::fprintf(csv_, ",distance_%d", i);
    std::fprintf(csv_, "\n");

    RCLCPP_INFO(get_logger(), "First way_point received, clock started. Logging to %s",
                run_dir_.c_str());
  }

  void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    if (state_ != State::Running) return;
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"), it_z(*msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
      voxels_.insert(voxelKey(*it_x, *it_y, *it_z));
    }
  }

  void odomHandler(int i, const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    auto & r = robots_[i];
    const auto & p = msg->pose.pose.position;
    if (r.has_pose && state_ == State::Running) {
      r.distance += std::sqrt((p.x - r.x) * (p.x - r.x) + (p.y - r.y) * (p.y - r.y) +
                              (p.z - r.z) * (p.z - r.z));
    }
    r.x = p.x; r.y = p.y; r.z = p.z;
    r.has_pose = true;
  }

  double volume() const
  {
    return voxel_size_ * voxel_size_ * voxel_size_ * static_cast<double>(voxels_.size());
  }

  void tick()
  {
    if (state_ != State::Running) return;

    const double t = std::min(elapsed(), time_limit_);
    const double vol = volume();

    std_msgs::msg::Float32 msg;
    msg.data = vol;
    pub_volume_->publish(msg);
    msg.data = t;
    pub_time_->publish(msg);

    std::fprintf(csv_, "%.1f,%.2f", t, vol);
    for (const auto & r : robots_) std::fprintf(csv_, ",%.2f", r.distance);
    std::fprintf(csv_, "\n");
    std::fflush(csv_);

    if (++tick_count_ % 2 == 0) publishVoxels();

    if (elapsed() >= time_limit_) finish();
  }

  void publishVoxels()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(voxels_.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
    const int64_t mask = (1 << 21) - 1, off = 1 << 20;
    for (int64_t key : voxels_) {
      *it_x = ((key >> 42 & mask) - off + 0.5) * voxel_size_;
      *it_y = ((key >> 21 & mask) - off + 0.5) * voxel_size_;
      *it_z = ((key & mask) - off + 0.5) * voxel_size_;
      ++it_x; ++it_y; ++it_z;
    }
    pub_voxels_->publish(cloud);
  }

  void finish()
  {
    state_ = State::Finished;
    std::fclose(csv_);
    csv_ = nullptr;

    double total = 0;
    for (const auto & r : robots_) total += r.distance;

    FILE * f = std::fopen((run_dir_ / "summary.txt").c_str(), "w");
    std::fprintf(f, "robots: %d\ntime_limit: %.1f\nexplored_volume: %.2f\ntotal_distance: %.2f\n",
                 n_robots_, time_limit_, volume(), total);
    for (int i = 0; i < n_robots_; i++) {
      std::fprintf(f, "distance_%d: %.2f\n", i, robots_[i].distance);
    }
    std::fclose(f);

    std_msgs::msg::Bool done;
    done.data = true;
    pub_finished_->publish(done);
    publishVoxels();

    RCLCPP_INFO(get_logger(), "Time limit reached. Team explored volume: %.2f m^3, "
                "total distance: %.2f m. Results in %s", volume(), total, run_dir_.c_str());
  }

  int n_robots_;
  double time_limit_;
  double voxel_size_;
  std::string log_dir_;

  std::vector<Robot> robots_;
  std::unordered_set<int64_t> voxels_;
  State state_ = State::Waiting;
  rclcpp::Time start_time_;
  fs::path run_dir_;
  FILE * csv_ = nullptr;
  int tick_count_ = 0;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_volume_, pub_time_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_finished_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxels_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Evaluator>());
  rclcpp::shutdown();
  return 0;
}
