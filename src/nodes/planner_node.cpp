#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "control/pure_pursuit.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "planning/astar.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  // TODO include other dimensions, because now it assumes the floor to be completely flat
  double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    double siny = 2 * q.w * q.z;
    double cosy = q.w * q.w - q.z * q.z;
    auto yaw = std::atan2(siny, cosy);
    return yaw;
  }

  class PlannerNode : public rclcpp::Node
  {
  public:
    PlannerNode() : Node("planner_node")
    {
      // ### planner node
      // input: occupancy grid, robot pose, goal
      // output: planned path

      // responsibility:
      //   convert world pose/goal to grid cells
      //   run A*
      //   convert grid path back to world coordinates
      //   replan when map/goal/pose changes enough
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
          });
      timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this]
                                 { on_timer(); });
      RCLCPP_INFO(get_logger(), "planner node started");
    }

  private:
    // TODO implement getting a goal from Gazebo
    toy_rover::control::Point2D goal{2.0, 2.0};
    void on_timer()
    {
      if (!latest_pose_)
      {
        return;
      }

      const toy_rover::control::Point2D world_start{latest_pose_->x, latest_pose_->y};
      const auto grid_start = world_to_grid(world_start);
      const auto grid_goal = world_to_grid(goal);

      std::vector<toy_rover::control::Point2D> world_path;

      auto route = planner_.plan(grid_, grid_start, grid_goal);
      if (!route)
      {
        return;
      }

      world_path.reserve(route->size());
      for (const auto cell : *route)
      {
        world_path.push_back(grid_to_world(cell));
      }

      // TODO send route to controller; controller should have actual route to execute on
    }

    void on_odometry(const nav_msgs::msg::Odometry &msg)
    {
      latest_pose_ = toy_rover::control::Pose2D{
          msg.pose.pose.position.x,
          msg.pose.pose.position.y,
          yaw_from_quaternion(msg.pose.pose.orientation),
      };
    }

    toy_rover::mapping::GridIndex world_to_grid(const toy_rover::control::Point2D &point) const
    {
      const double resolution = grid_.resolution_m();
      return toy_rover::mapping::GridIndex{
          static_cast<int>(std::floor(point.x / resolution)),
          static_cast<int>(std::floor(point.y / resolution)),
      };
    }

    toy_rover::control::Point2D grid_to_world(toy_rover::mapping::GridIndex index) const
    {
      const double resolution = grid_.resolution_m();
      return toy_rover::control::Point2D{
          (static_cast<double>(index.x) + 0.5) * resolution,
          (static_cast<double>(index.y) + 0.5) * resolution,
      };
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    toy_rover::mapping::OccupancyGrid grid_{80, 80, 0.1};
    toy_rover::planning::AStar planner_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
