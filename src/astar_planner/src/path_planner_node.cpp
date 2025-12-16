#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <fstream>

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
    // =========================
    // - obstacle_threshold: 벽 판정 더 보수적으로
    // - inflation_radius: 벽을 더 두껍게(셀 단위)
    // =========================
    this->declare_parameter<int>("obstacle_threshold", 30);
    this->declare_parameter<int>("inflation_radius", 3); 

    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
    inflation_radius_   = this->get_parameter("inflation_radius").as_int();

    //기존에 있던 resolution 파라미터는 실제로 map_msg_의 resolution을 쓰고 있어서 의미가 거의 없음
    this->declare_parameter<double>("resolution", 1.0);
    resolution_ = this->get_parameter("resolution").as_double();

    //Initialize
    has_map_ = false;
    has_goal_ = false;
    has_current_pose_ = false;
    goal_reached_ = false;

    //Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 
      rclcpp::QoS(rclcpp::KeepLast(1))
        .transient_local()
        .reliable(),
      std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/go1_pose", 10,
      std::bind(&PathPlannerNode::currentPoseCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

    // Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 10);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized");
    RCLCPP_INFO(this->get_logger(), "Use RViz2 '2D Goal Pose' tool to set a goal");

    RCLCPP_INFO(this->get_logger(),
      "Params: obstacle_threshold=%d, inflation_radius=%d",
      obstacle_threshold_, inflation_radius_);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;

    int width = msg->info.width;
    int height = msg->info.height;

    //threshold를 50 고정에서 -> 파라미터 obstacle_threshold_ 사용
    //inflation을 위해 먼저 원본 binary 맵을 만들고, 그 다음 팽창 맵을 생성
    std::vector<std::vector<int>> raw_grid(height, std::vector<int>(width, 0));
    map_grid_.assign(height, std::vector<int>(width, 0));

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;

        //obstacle_threshold_ 이거나 unknown이면 obstacle
        if (msg->data[index] > obstacle_threshold_ || msg->data[index] < 0) {
          raw_grid[y][x] = 1;
        } else {
          raw_grid[y][x] = 0;
        }
      }
        }

    //장애물 인플레이션(벽 두껍게)
    int R = std::max(0, inflation_radius_);
    int R2 = R * R;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (raw_grid[y][x] == 1) {
          for (int dy = -R; dy <= R; ++dy) {
            int ny = y + dy;
            if (ny < 0 || ny >= height) continue;
            for (int dx = -R; dx <= R; ++dx) {
              int nx = x + dx;
              if (nx < 0 || nx >= width) continue;

              // ✅ 핵심: 원형
              if (dx * dx + dy * dy <= R2) {
                map_grid_[ny][nx] = 1;
              }
            }
          }
        }
      }
    }

    //Distance transform (BFS) -> cost_map 생성
    std::vector<std::vector<int>> dist(height, std::vector<int>(width, 1e9));
    std::deque<std::pair<int,int>> q;
    //obstacle을 seed로 큐에 넣기
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (map_grid_[y][x] == 1) {
          dist[y][x] = 0;
          q.push_back({y, x});
        }
      }
    }
    //4-neighbor BFS로 최단거리 확산
    const int dy4[4] = {1,-1,0,0};
    const int dx4[4] = {0,0,1,-1};

    while (!q.empty()) {
      auto [cy, cx] = q.front();
      q.pop_front();

      for (int k = 0; k < 4; ++k) {
        int ny = cy + dy4[k];
        int nx = cx + dx4[k];
        if (ny < 0 || ny >= height || nx < 0 || nx >= width) continue;

        if (dist[ny][nx] > dist[cy][cx] + 1) {
          dist[ny][nx] = dist[cy][cx] + 1;
          q.push_back({ny, nx});
        }
      }
    }

    //dist -> cost_map (벽 가까울수록 비용 크게)
    std::vector<std::vector<float>> cost_map(height, std::vector<float>(width, 1.0f));
    const int safe_cells = 6;
    const float w = 5.0f;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (map_grid_[y][x] == 1) {
          cost_map[y][x] = 1e9f;
          continue;
        }

        int d = dist[y][x];
        if (d < safe_cells) {
          float t = static_cast<float>(safe_cells - d);
          cost_map[y][x] = 1.0f + w * t * t;
        } else {
          cost_map[y][x] = 1.0f;
        }
      }
    }


    astar_.setMap(map_grid_);
    astar_.setCostMap(cost_map);

    if (!has_map_) {
      has_map_ = true;
      RCLCPP_INFO(this->get_logger(), "Map received: %dx%d", width, height);
      RCLCPP_INFO(this->get_logger(), "Inflation applied: radius=%d cells", R);
    }
  }

  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_current_pose_) {
      has_current_pose_ = true;
      current_pose_ = *msg;
      previous_pose_ = *msg;
      RCLCPP_INFO(this->get_logger(), "Robot position initialized at (%.2f, %.2f)",
        current_pose_.pose.position.x, current_pose_.pose.position.y);
      return;
    }

    double dx = msg->pose.position.x - previous_pose_.pose.position.x;
    double dy = msg->pose.position.y - previous_pose_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.01) {
      return;
    }

    current_pose_ = *msg;

    if (has_goal_) {
      double goal_dx = current_pose_.pose.position.x - goal_pose_.pose.position.x;
      double goal_dy = current_pose_.pose.position.y - goal_pose_.pose.position.y;
      double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);

      if (goal_distance < 0.5) {
        if (!goal_reached_) {
          RCLCPP_INFO(this->get_logger(), "✓ Goal reached!");
          goal_reached_ = true;
        }
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Robot moved to (%.2f, %.2f)",
      current_pose_.pose.position.x, current_pose_.pose.position.y);

    previous_pose_ = current_pose_;

    if (has_map_ && has_goal_ && !goal_reached_) {
      replanPath();
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    goal_reached_ = false;

    RCLCPP_INFO(this->get_logger(),
      "New goal received: (%.2f, %.2f)",
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);

    publishGoalMarker();

    if (has_map_ && has_current_pose_) {
      replanPath();
    }
  }

  void replanPath()
  {
    if (!has_map_ || !has_current_pose_ || !has_goal_) {
      return;
    }

    astar_planner::GridCell start = worldToGrid(
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);

    astar_planner::GridCell goal = worldToGrid(
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);

    auto path_cells = astar_.findPath(start, goal);

    if (path_cells.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found!");
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.header.stamp = this->now();
    first_pose.header.frame_id = "map";
    first_pose.pose = current_pose_.pose;
    path_msg.poses.push_back(first_pose);

    for (size_t i = 0; i < path_cells.size(); ++i) {
      const auto& cell = path_cells[i];

      auto world_pos = gridToWorld(cell.x, cell.y);

      double dx = world_pos.first - current_pose_.pose.position.x;
      double dy = world_pos.second - current_pose_.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (i == 0 && dist < 0.3) {
        continue;
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = world_pos.first;
      pose.pose.position.y = world_pos.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    publishPathMarkers(path_cells);

    static size_t last_path_size = 0;
    if (last_path_size == 0 || std::abs((int)path_cells.size() - (int)last_path_size) > 3) {
      RCLCPP_INFO(this->get_logger(), "Path updated: %zu waypoints", path_cells.size());
      last_path_size = path_cells.size();
    }

    savePathToFile(path_cells);
  }

  astar_planner::GridCell worldToGrid(double x, double y)
  {
    astar_planner::GridCell cell;

    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double resolution = map_msg_->info.resolution;

    cell.x = static_cast<int>((x - origin_x) / resolution);
    cell.y = static_cast<int>((y - origin_y) / resolution);

    return cell;
  }

  std::pair<double, double> gridToWorld(int x, int y)
  {
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    double resolution = map_msg_->info.resolution;

    double world_x = origin_x + (x + 0.5) * resolution;
    double world_y = origin_y + (y + 0.5) * resolution;

    return {world_x, world_y};
  }

  void publishPathMarkers(const std::vector<astar_planner::GridCell>& path)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.1;

    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    for (const auto& cell : path) {
      geometry_msgs::msg::Point p;
      auto world_pos = gridToWorld(cell.x, cell.y);
      p.x = world_pos.first;
      p.y = world_pos.second;
      p.z = 0.1;
      line_marker.points.push_back(p);
    }

    marker_array.markers.push_back(line_marker);
    viz_pub_->publish(marker_array);
  }
  void savePathToFile(const std::vector<astar_planner::GridCell>& path)
  {
    std::ofstream file("/tmp/path.txt");
    for (const auto& cell : path) {
      auto wp = gridToWorld(cell.x, cell.y);
      file << wp.first << " " << wp.second << "\n";
    }
    file.close();
  }
  void publishGoalMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal_pose_.pose.position.x;
    marker.pose.position.y = goal_pose_.pose.position.y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    goal_marker_pub_->publish(marker);
  }

  // ROS objects
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

  // State variables
  bool has_map_;
  bool has_goal_;
  bool has_current_pose_;
  bool goal_reached_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped previous_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  std::vector<std::vector<int>> map_grid_;
  astar_planner::AStar astar_;

  // Parameters
  double resolution_;
  int obstacle_threshold_;   // ✅ [추가]
  int inflation_radius_;     // ✅ [추가]
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}