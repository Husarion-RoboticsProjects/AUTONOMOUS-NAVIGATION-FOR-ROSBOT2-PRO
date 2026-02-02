#include "global_motion/global_motion_node.hpp"
#include <algorithm>
#include <chrono>
#include <queue>
#include <map>


GlobalMotionNode::GlobalMotionNode()
: Node("global_motion_node")
{
  rclcpp::QoS map_qos(1);
  map_qos.transient_local();
  map_qos.reliable();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos, std::bind(&GlobalMotionNode::mapCallback, this, std::placeholders::_1));

  start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10, std::bind(&GlobalMotionNode::startCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&GlobalMotionNode::goalCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  path_pub_ = create_publisher<visualization_msgs::msg::Marker>("planned_path", 10);

  path_pub_real_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);

  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 
    10, 
    std::bind(&GlobalMotionNode::amclCallback, this, std::placeholders::_1)

  );

// QoS for sensors

auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10));
sensor_qos.best_effort();
sensor_qos.durability_volatile();

  sub_fl_ = create_subscription<sensor_msgs::msg::Range>("/range/fl", sensor_qos, std::bind(&GlobalMotionNode::cb_fl, this, std::placeholders::_1));
  sub_fr_ = create_subscription<sensor_msgs::msg::Range>("/range/fr", sensor_qos, std::bind(&GlobalMotionNode::cb_fr, this, std::placeholders::_1));
  sub_rl_ = create_subscription<sensor_msgs::msg::Range>("/range/rl", sensor_qos, std::bind(&GlobalMotionNode::cb_rl, this, std::placeholders::_1));
  sub_rr_ = create_subscription<sensor_msgs::msg::Range>("/range/rr", sensor_qos, std::bind(&GlobalMotionNode::cb_rr, this, std::placeholders::_1));
  

  RCLCPP_INFO(this->get_logger(), "/global_plan Publisher ready");

  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&GlobalMotionNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Global Motion Node ready");
}



struct AStarNode {
    int x, y;
    float f; // f = g + h
    bool operator>(const AStarNode& other) const { return f > other.f; }
};

void GlobalMotionNode::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{

    current_pose_ = msg->pose.pose;
    localization_ready_ = true;

    // RCLCPP_DEBUG(get_logger(), "Updated Pose: x=%.2f, y=%.2f", 
    //              current_pose_.position.x, current_pose_.position.y);
}

std::vector<GridNode> GlobalMotionNode::aStar(int sx, int sy, int gx, int gy)
{
    int width = map_.info.width;
    int height = map_.info.height;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::vector<float> g_costs(width * height, std::numeric_limits<float>::infinity());
    std::vector<int> parent_map(width * height, -1); // for Backtracking

    // Init
    int start_idx = sy * width + sx;
    g_costs[start_idx] = 0;
    open_list.push({sx, sy, (float)hypot(gx - sx, gy - sy)});

    bool found = false;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.x == gx && current.y == gy) {
            found = true;
            break;
        }

        // Explore 8 neighbours
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int nx = current.x + dx;
                int ny = current.y + dy;
                int n_idx = ny * width + nx;

                if (isFree(nx, ny)) {
                    float move_cost = hypot(dx, dy);
                    float new_g = g_costs[current.y * width + current.x] + move_cost;

                    if (new_g < g_costs[n_idx]) {
                        g_costs[n_idx] = new_g;
                        parent_map[n_idx] = current.y * width + current.x;
                        float h = hypot(gx - nx, gy - ny);
                        open_list.push({nx, ny, new_g + h});
                    }
                }
            }
        }
    }

    // 2.(Backtracking)
    std::vector<GridNode> path;
    if (found) {
        int curr = gy * width + gx;
        while (curr != -1) {
            path.push_back({curr % width, curr / width, 0, 0, -1, -1});
            curr = parent_map[curr];
        }
        std::reverse(path.begin(), path.end());
    } else {
        RCLCPP_ERROR(get_logger(), "No Path Found");
    }

    return path;
}



// ---------------- CALLBACKS ---------------

void GlobalMotionNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  map_ready_ = true;

  RCLCPP_WARN(get_logger(), "MAP received [%dx%d]",
    map_.info.width, map_.info.height);
}

void GlobalMotionNode::startCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  start_pose_ = msg->pose.pose;
  start_ready_ = true;

  RCLCPP_WARN(get_logger(), "START received (%.2f, %.2f)",
    start_pose_.position.x,
    start_pose_.position.y);
}

void GlobalMotionNode::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = msg->pose;
  goal_ready_ = true;

  RCLCPP_WARN(get_logger(), "GOAL received (%.2f, %.2f)",
    goal_pose_.position.x,
    goal_pose_.position.y);

  RCLCPP_WARN(get_logger(),
    "GOAL received | map=%d start=%d",
    map_ready_, start_ready_);

  if (map_ready_ && start_ready_) {
    RCLCPP_WARN(get_logger(), "Calling planPath()");
    planPath();
  }
}


// ---------------- PLANNING ----------------
void GlobalMotionNode::publishPathMarker()
{
    if(path_.empty()) return;

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = now();
    line.ns = "global_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for(auto &node : path_)
    {
        geometry_msgs::msg::Point p;
        p.x = node.x * map_.info.resolution + map_.info.origin.position.x;
        p.y = node.y * map_.info.resolution + map_.info.origin.position.y;
        p.z = 0.0;
        line.points.push_back(p);
    }

    path_pub_->publish(line);
}


bool GlobalMotionNode::planPath()
{
  int sx, sy, gx, gy;
  if (!worldToGrid(start_pose_.position.x, start_pose_.position.y, sx, sy)) return false;
  if (!worldToGrid(goal_pose_.position.x, goal_pose_.position.y, gx, gy)) return false;

  RCLCPP_INFO(get_logger(), "Start grid: %d %d, Goal grid: %d %d", sx, sy, gx, gy);

  RCLCPP_INFO(this->get_logger(), "Checking Start (%d, %d): %s", sx, sy, isFree(sx, sy) ? "FREE" : "OCCUPIED");
  RCLCPP_INFO(this->get_logger(), "Checking Goal (%d, %d): %s", gx, gy, isFree(gx, gy) ? "FREE" : "OCCUPIED");
  path_ = aStar(sx, sy, gx, gy);
  current_target_ = 0;
  if (path_.empty()) {
        RCLCPP_ERROR(get_logger(), "No path found");
        return false;
    }

  
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "map"; // 

  for (const auto &node : path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = path_msg.header.stamp;
      pose.header.frame_id = "map";

      // From pixels (grid) to meters
      pose.pose.position.x = node.x * map_.info.resolution + map_.info.origin.position.x;
      pose.pose.position.y = node.y * map_.info.resolution + map_.info.origin.position.y;
      pose.pose.position.z = 0.0;

      // Initial orientation
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;

      path_msg.poses.push_back(pose);
  }


  // Publish real Path
  path_pub_real_->publish(path_msg);
  
  // Marker visual (para RViz)
  publishPathMarker(); 
  
  RCLCPP_INFO(get_logger(), "Published path: %zu points", path_msg.poses.size());
  return true;
  
}

bool GlobalMotionNode::worldToGrid(double wx, double wy, int &gx, int &gy)
{
  gx = (wx - map_.info.origin.position.x) / map_.info.resolution;
  gy = (wy - map_.info.origin.position.y) / map_.info.resolution;

  return gx >= 0 && gy >= 0 &&
         gx < (int)map_.info.width &&
         gy < (int)map_.info.height;
}

bool GlobalMotionNode::isFree(int x, int y)
{
    // 1. Define safety radius (inflation layer in cells)
    
    int margin = 3; 

    for (int dx = -margin; dx <= margin; dx++) {
        for (int dy = -margin; dy <= margin; dy++) {
            int nx = x + dx;
            int ny = y + dy;

            // 2. Map boundary check for each neighbor
            if (nx < 0 || nx >= (int)map_.info.width || ny < 0 || ny >= (int)map_.info.height) {
                return false; 
            }

            // 3. Standard ROS2 foormula 
            int idx = ny * map_.info.width + nx;

            int value = (int)map_.data[idx];

            // 4. Occupancy logic
            // Identified as obstacle if:
            // - Wall (100)
            // - Unknown area (-1) -> MUY IMPORTANTE
            // - Mid-range occupancy probability (>20)
            if (value == -1 || value > 20) {
                return false; 
            }
        }
    }

    // If cell and neighbors are clear:
    return true;
}


// ---------------- CONTROL ----------------

void GlobalMotionNode::controlLoop()
{
    // 1. Wait for first amcl data
    if (!localization_ready_ || path_.empty() || current_target_ >= path_.size()) {
        publishCmd(0.0, 0.0);
        return;
    }

    
    RCLCPP_INFO(get_logger(), "SENSORS -> fl=%.3fm, fr=%.3fm, rl=%.3fm, rr=%.3fm", 
                fl_, fr_, rl_, rr_);

    float threshold = 0.30; // 30 cm
    geometry_msgs::msg::Twist cmd;

    // 3. Front Obstacle Avoidance Logic
    if (std::min(fl_, fr_) < threshold) {
        cmd.linear.x = 0.0; // detener avance
        
        // Tuurn towards the side with more clearance
        if (fl_ > fr_) {
            cmd.angular.z = 0.6; // Turn left (positive in ROS)
            RCLCPP_WARN(get_logger(), "Obstacle front - turning LEFT");
        } else {
            cmd.angular.z = -0.6; // Turn right (negative in ROS)
            RCLCPP_WARN(get_logger(), "Obstacle front - turning RIGHT");
        }
        
        // Publish avoidance maneuver
        cmd_pub_->publish(cmd);
        return; 
    } 
    
    // 4. If path is clear, proceed with A* navigation
    else {
        // Get goal pose
        auto target_node = path_[current_target_];
        double tx = target_node.x * map_.info.resolution + map_.info.origin.position.x;
        double ty = target_node.y * map_.info.resolution + map_.info.origin.position.y;

        double dx = tx - current_pose_.position.x;
        double dy = ty - current_pose_.position.y;
        double distance = std::hypot(dx, dy);
        double angle_to_target = std::atan2(dy, dx);

        // Get current Yaw
        tf2::Quaternion q(current_pose_.orientation.x, current_pose_.orientation.y,
                          current_pose_.orientation.z, current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, current_yaw;
        m.getRPY(r, p, current_yaw);

        double angular_error = angle_to_target - current_yaw;
        while (angular_error > M_PI) angular_error -= 2.0 * M_PI;
        while (angular_error < -M_PI) angular_error += 2.0 * M_PI;

        if (distance < 0.20) {
            current_target_++;
            RCLCPP_INFO(get_logger(), "Next waypoint: %zu", current_target_);
        } else {
            // Normal linear veclocity
            double lin_vel = 0.24; 
            
            // If orientation error exceeds, perform in-place rotation
            if (std::abs(angular_error) > 0.8) {
                lin_vel = 0.0;
            }
            
            publishCmd(lin_vel, angular_error * 0.7);
        }
    }
}

void GlobalMotionNode::publishCmd(double lin, double ang)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = lin;
  cmd.angular.z = ang;
  cmd_pub_->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
