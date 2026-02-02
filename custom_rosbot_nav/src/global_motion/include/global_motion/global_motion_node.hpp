#ifndef GLOBAL_MOTION_NODE_HPP
#define GLOBAL_MOTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/range.hpp>


struct GridNode {
  int x, y;
  float g, h;
  int parent_x, parent_y;
};

class GlobalMotionNode : public rclcpp::Node
{
public:
  GlobalMotionNode();

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void cb_fl(const sensor_msgs::msg::Range::SharedPtr msg) { fl_ = msg->range; }
  void cb_fr(const sensor_msgs::msg::Range::SharedPtr msg) { fr_ = msg->range; }
  void cb_rl(const sensor_msgs::msg::Range::SharedPtr msg) { rl_ = msg->range; }
  void cb_rr(const sensor_msgs::msg::Range::SharedPtr msg) { rr_ = msg->range; }
  
  // Planning
  bool planPath();
  std::vector<GridNode> aStar(int sx, int sy, int gx, int gy);
  
  // Control
  void controlLoop();
  void publishCmd(double lin, double ang);

  // Utils
  bool worldToGrid(double wx, double wy, int &gx, int &gy);
  bool isFree(int x, int y);

  // Visualization
  void publishPathMarker();

  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_real_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_fl_, sub_fr_, sub_rl_, sub_rr_;
  // State
  nav_msgs::msg::OccupancyGrid map_;
  bool map_ready_ = false;
  bool start_ready_ = false;
  bool goal_ready_ = false;
  bool localization_ready_ = false;
  
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Pose current_pose_;

  std::vector<GridNode> path_;
  size_t current_target_ = 0;

  float fl_ = 1.0, fr_ = 1.0, rl_ = 1.0, rr_ = 1.0;
  float obstacle_threshold_ = 0.30; // 30 cm
};

#endif
