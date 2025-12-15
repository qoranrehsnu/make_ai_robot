// Fill in the code here!#include <memory>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "astar_planner/astar.hpp"

using namespace std::chrono_literals;

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : Node("path_planner_node")
  {
    // Parameters
    this->declare_parameter<double>("resolution", 1.0);
    this->declare_parameter<int>("inflate_cells", 3);

    resolution_     = this->get_parameter("resolution").as_double();
    inflate_cells_  = this->get_parameter("inflate_cells").as_int();

    has_map_ = false;
    has_goal_ = false;
    has_current_pose_ = false;
    goal_reached_ = false;

    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/go1_pose", 10,
      std::bind(&PathPlannerNode::currentPoseCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    viz_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
    goal_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

    RCLCPP_INFO(this->get_logger(),
      "Path Planner initialized (inflate_cells=%d)", inflate_cells_);
  }

private:
  /* ===============================
     MAP CALLBACK (누적 inflation 해결)
     =============================== */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;

    int width  = msg->info.width;
    int height = msg->info.height;

    /* 1. OccupancyGrid → raw_map_ (원본) */
    raw_map_.assign(height, std::vector<int>(width, 0));

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        raw_map_[y][x] =
          (msg->data[idx] > 50 || msg->data[idx] < 0) ? 1 : 0;
      }
    }

    /* 2. fresh copy → map_grid_ */
    map_grid_ = raw_map_;

    /* 3. inflation (항상 1회) */
    inflateObstacles(inflate_cells_);

    /* 4. A* map 설정 */
    astar_.setMap(map_grid_);

    if (!has_map_) {
      has_map_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Map received: %dx%d (inflation=%d cells)",
        width, height, inflate_cells_);
    }
  }

  /* ===============================
     OBSTACLE INFLATION
     =============================== */
  void inflateObstacles(int radius)
  {
    if (radius <= 0) return;

    int height = map_grid_.size();
    int width  = map_grid_[0].size();

    std::vector<std::pair<int,int>> obstacles;
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        if (map_grid_[y][x] == 1)
          obstacles.push_back({x, y});

    for (const auto& obs : obstacles) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx*dx + dy*dy > radius*radius) continue;

          int nx = obs.first  + dx;
          int ny = obs.second + dy;

          if (nx >= 0 && nx < width && ny >= 0 && ny < height)
            map_grid_[ny][nx] = 1;
        }
      }
    }
  }

  /* ===============================
     CURRENT POSE CALLBACK
     =============================== */
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_current_pose_) {
      current_pose_ = *msg;
      previous_pose_ = *msg;
      has_current_pose_ = true;
      return;
    }

    double dx = msg->pose.position.x - previous_pose_.pose.position.x;
    double dy = msg->pose.position.y - previous_pose_.pose.position.y;
    if (std::sqrt(dx*dx + dy*dy) < 0.01) return;

    current_pose_ = *msg;
    previous_pose_ = *msg;

    if (has_map_ && has_goal_ && !goal_reached_)
      replanPath();
  }

  /* ===============================
     GOAL CALLBACK
     =============================== */
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    goal_reached_ = false;

    publishGoalMarker();

    if (has_map_ && has_current_pose_)
      replanPath();
  }

  /* ===============================
     REPLAN
     =============================== */
  void replanPath()
  {
    astar_planner::GridCell start =
      worldToGrid(current_pose_.pose.position.x,
                  current_pose_.pose.position.y);

    astar_planner::GridCell goal =
      worldToGrid(goal_pose_.pose.position.x,
                  goal_pose_.pose.position.y);

    auto path_cells = astar_.findPath(start, goal);
    if (path_cells.empty()) return;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();

    for (const auto& c : path_cells) {
      auto wp = gridToWorld(c.x, c.y);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = wp.first;
      ps.pose.position.y = wp.second;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }

    path_pub_->publish(path);
    publishPathMarkers(path_cells);
  }

  /* ===============================
     COORDINATE CONVERSION
     =============================== */
  astar_planner::GridCell worldToGrid(double x, double y)
  {
    astar_planner::GridCell c;
    c.x = static_cast<int>((x - map_msg_->info.origin.position.x)
                            / map_msg_->info.resolution);
    c.y = static_cast<int>((y - map_msg_->info.origin.position.y)
                            / map_msg_->info.resolution);
    return c;
  }

  std::pair<double,double> gridToWorld(int x, int y)
  {
    double ox = map_msg_->info.origin.position.x;
    double oy = map_msg_->info.origin.position.y;
    double r  = map_msg_->info.resolution;
    return {ox + (x + 0.5) * r, oy + (y + 0.5) * r};
  }

  /* ===============================
     VISUALIZATION
     =============================== */
  void publishPathMarkers(
    const std::vector<astar_planner::GridCell>& path)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.1;
    m.color.g = 1.0;
    m.color.a = 1.0;

    for (const auto& c : path) {
      auto wp = gridToWorld(c.x, c.y);
      geometry_msgs::msg::Point p;
      p.x = wp.first;
      p.y = wp.second;
      p.z = 0.1;
      m.points.push_back(p);
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(m);
    viz_pub_->publish(arr);
  }

  void publishGoalMarker()
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose = goal_pose_.pose;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color.b = 1.0;
    m.color.a = 0.8;
    goal_marker_pub_->publish(m);
  }

  /* ===============================
     ROS OBJECTS & STATE
     =============================== */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

  bool has_map_, has_goal_, has_current_pose_, goal_reached_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  geometry_msgs::msg::PoseStamped current_pose_, previous_pose_, goal_pose_;

  std::vector<std::vector<int>> raw_map_;   // 원본 맵
  std::vector<std::vector<int>> map_grid_;  // inflation용 맵

  astar_planner::AStar astar_;

  double resolution_;
  int inflate_cells_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

