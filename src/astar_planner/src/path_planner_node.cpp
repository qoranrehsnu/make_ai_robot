#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "astar_planner/astar.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode()
    : Node("path_planner_node")
    {
        declare_parameter<double>("resolution", 1.0);
        declare_parameter<double>("robot_radius", 0.3);

        get_parameter("resolution", resolution_);
        get_parameter("robot_radius", robot_radius_);

        inflation_cells_ =
            static_cast<int>(std::ceil(robot_radius_ / resolution_));

        RCLCPP_INFO(get_logger(),
            "robot_radius=%.2f m → inflation=%d cells",
            robot_radius_, inflation_cells_);

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>("local_path", 10);

        // 예시 map (실제로는 simulator_node에서 수신)
        grid_map_ = {
            {0,0,0,0,0,0},
            {0,1,1,1,0,0},
            {0,0,0,1,0,0},
            {0,1,0,0,0,0},
            {0,0,0,0,0,0}
        };

        // ★ 핵심: obstacle inflation
        AStar::inflateObstacles(grid_map_, inflation_cells_);

        plan();
    }

private:
    void plan()
    {
        AStar::Point start{0, 0};
        AStar::Point goal{5, 4};

        auto path = astar_.search(start, goal, grid_map_);

        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";

        for (auto& p : path)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.pose.position.x = p.first * resolution_;
            ps.pose.position.y = p.second * resolution_;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }

        path_pub_->publish(msg);
    }

    double resolution_;
    double robot_radius_;
    int inflation_cells_;

    AStar astar_;
    AStar::Grid grid_map_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}

