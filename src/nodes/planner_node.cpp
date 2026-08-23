#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "control/pure_pursuit.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mapping/obstacle_inflation.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planning/astar.hpp"
#include "planning/goal_resolver.hpp"
#include "planning/path_simplifier.hpp"
#include "planning/start_escape.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

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
    PlannerNode() : Node("planner_node"),
                    obstacle_inflation_radius_m_(declare_parameter<double>(
                        "obstacle_inflation_radius_m", 0.33)),
                    escape_inflation_radius_m_(declare_parameter<double>(
                        "escape_inflation_radius_m", 0.295)),
                    goal_snap_radius_m_(declare_parameter<double>(
                        "goal_snap_radius_m", 1.0)),
                    path_publisher_(create_publisher<nav_msgs::msg::Path>("planned_path", 10)),
                    recovery_publisher_(create_publisher<std_msgs::msg::Empty>("unblock", 10))
    {
      if (escape_inflation_radius_m_ < 0.0 ||
          escape_inflation_radius_m_ > obstacle_inflation_radius_m_)
      {
        throw std::invalid_argument(
            "escape inflation radius must be non-negative and no greater than normal inflation");
      }
      if (goal_snap_radius_m_ < 0.0)
      {
        throw std::invalid_argument("goal snap radius must be non-negative");
      }

      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
          });

      grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map",
          10,
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
          {
            on_map(*msg);
          });

      goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          "goal_pose",
          10,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            on_goal_pose(*msg);
          });

      timer_ = create_wall_timer(std::chrono::milliseconds{200}, [this]
                                 { on_timer(); });
      RCLCPP_INFO(get_logger(), "planner node started");
    }

  private:
    toy_rover::control::Point2D goal_{-6.3, -6.1};
    void on_timer()
    {
      if (!latest_pose_ || !has_map_)
      {
        return;
      }

      const toy_rover::control::Point2D world_start{latest_pose_->x, latest_pose_->y};
      const auto grid_start = world_to_grid(world_start);
      const auto requested_grid_goal = world_to_grid(goal_);
      auto planning_start = grid_start;
      std::optional<toy_rover::planning::Path> escape;
      if (grid_.in_bounds(grid_start) &&
          grid_.at(grid_start) == toy_rover::mapping::Cell::Occupied)
      {
        escape = toy_rover::planning::find_path_out_of_inflation(
            escape_grid_, grid_, grid_start);
        if (escape)
        {
          planning_start = escape->back();
        }
      }

      auto grid_goal = requested_grid_goal;
      std::optional<toy_rover::planning::Path> continuation;
      if (adjusted_goal_)
      {
        const auto adjusted_grid_goal = world_to_grid(*adjusted_goal_);
        if (grid_.in_bounds(adjusted_grid_goal) &&
            grid_.at(adjusted_grid_goal) != toy_rover::mapping::Cell::Occupied)
        {
          grid_goal = adjusted_grid_goal;
          continuation = planner_.plan(grid_, planning_start, grid_goal);
        }
        else
        {
          adjusted_goal_.reset();
        }
      }

      if (!adjusted_goal_ && grid_.in_bounds(requested_grid_goal) &&
          grid_.at(requested_grid_goal) == toy_rover::mapping::Cell::Occupied)
      {
        auto resolved_goal = toy_rover::planning::resolve_reachable_goal(
            grid_, planning_start, requested_grid_goal, goal_snap_radius_m_);
        if (resolved_goal)
        {
          grid_goal = resolved_goal->cell;
          continuation = std::move(resolved_goal->path);
          const auto adjusted_world_goal = grid_to_world(grid_goal);
          adjusted_goal_ = adjusted_world_goal;
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Requested goal is occupied; planning to nearest reachable free point "
              "x=%.2f y=%.2f",
              adjusted_world_goal.x, adjusted_world_goal.y);
        }
      }
      else if (!adjusted_goal_ && grid_.in_bounds(requested_grid_goal))
      {
        continuation = planner_.plan(grid_, planning_start, grid_goal);
      }

      std::vector<toy_rover::control::Point2D> world_path;
      std::optional<toy_rover::planning::Path> route;
      std::size_t escape_size = 0;
      if (continuation)
      {
        if (escape)
        {
          auto simplified_escape =
              toy_rover::planning::simplify_path(escape_grid_, *escape);
          const auto simplified_continuation =
              toy_rover::planning::simplify_path(grid_, *continuation);
          escape_size = simplified_escape.size();
          simplified_escape.insert(
              simplified_escape.end(),
              std::next(simplified_continuation.begin()),
              simplified_continuation.end());
          route = std::move(simplified_escape);
        }
        else if (grid_.in_bounds(grid_start) &&
                 grid_.at(grid_start) != toy_rover::mapping::Cell::Occupied)
        {
          route = toy_rover::planning::simplify_path(grid_, *continuation);
        }
      }
      if (!route)
      {
        path_publisher_->publish(make_path_message({})); // send an empty msg as a signal to stop
        request_local_recovery();
        const bool start_occupied =
            grid_.in_bounds(grid_start) &&
            grid_.at(grid_start) == toy_rover::mapping::Cell::Occupied;
        const bool goal_occupied =
            grid_.in_bounds(requested_grid_goal) &&
            grid_.at(requested_grid_goal) == toy_rover::mapping::Cell::Occupied;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No route; publishing stop (start in bounds=%d occupied=%d, "
            "goal in bounds=%d occupied=%d)",
            grid_.in_bounds(grid_start), start_occupied,
            grid_.in_bounds(grid_goal), goal_occupied);
        return;
      }

      if (escape)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rover is inside normal obstacle inflation; using %zu-point escape segment",
            escape_size);
      }
      world_path.reserve(route->size());
      for (const auto cell : *route)
      {
        world_path.push_back(grid_to_world(cell));
      }

      path_publisher_->publish(make_path_message(world_path));
    }

    void request_local_recovery()
    {
      const auto timestamp = std::chrono::steady_clock::now();
      constexpr auto request_interval = std::chrono::seconds{3};
      if (last_recovery_request_ &&
          timestamp - *last_recovery_request_ < request_interval)
      {
        return;
      }

      last_recovery_request_ = timestamp;
      recovery_publisher_->publish(std_msgs::msg::Empty{});
      RCLCPP_WARN(get_logger(), "No global route; requesting local free-space escape");
    }

    void on_odometry(const nav_msgs::msg::Odometry &msg)
    {
      latest_pose_ = toy_rover::control::Pose2D{
          msg.pose.pose.position.x,
          msg.pose.pose.position.y,
          yaw_from_quaternion(msg.pose.pose.orientation),
      };
    }

    void on_goal_pose(const geometry_msgs::msg::PoseStamped &msg)
    {
      set_goal(msg.pose.position.x, msg.pose.position.y);
    }

    void set_goal(double x, double y)
    {
      goal_ = toy_rover::control::Point2D{x, y};
      adjusted_goal_.reset();
      RCLCPP_INFO(get_logger(), "planner goal set to x=%.2f y=%.2f", goal_.x, goal_.y);
    }

    void on_map(const nav_msgs::msg::OccupancyGrid &msg)
    {
      if (msg.info.width != static_cast<std::uint32_t>(grid_.width()) ||
          msg.info.height != static_cast<std::uint32_t>(grid_.height()) ||
          std::abs(static_cast<double>(msg.info.resolution) - grid_.resolution_m()) > 1e-6 ||
          msg.data.size() != grid_.cells().size())
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "ignoring map with dimensions/resolution that do not match planner grid");
        return;
      }

      grid_origin_ = toy_rover::control::Point2D{
          msg.info.origin.position.x,
          msg.info.origin.position.y,
      };

      for (int y = 0; y < grid_.height(); ++y)
      {
        for (int x = 0; x < grid_.width(); ++x)
        {
          const auto index = toy_rover::mapping::GridIndex{x, y};
          const auto value = msg.data[grid_.linear_index(index)];
          if (value >= static_cast<std::int8_t>(toy_rover::mapping::Cell::Occupied))
          {
            grid_.set(index, toy_rover::mapping::Cell::Occupied);
          }
          else if (value == static_cast<std::int8_t>(toy_rover::mapping::Cell::Free))
          {
            grid_.set(index, toy_rover::mapping::Cell::Free);
          }
          else
          {
            grid_.set(index, toy_rover::mapping::Cell::Unknown);
          }
        }
      }

      escape_grid_ = grid_;
      toy_rover::mapping::inflate_occupied_cells(
          escape_grid_, escape_inflation_radius_m_);
      toy_rover::mapping::inflate_occupied_cells(grid_, obstacle_inflation_radius_m_);
      has_map_ = true;
    }

    toy_rover::mapping::GridIndex world_to_grid(const toy_rover::control::Point2D &point) const
    {
      const double resolution = grid_.resolution_m();
      return toy_rover::mapping::GridIndex{
          static_cast<int>(std::floor((point.x - grid_origin_.x) / resolution)),
          static_cast<int>(std::floor((point.y - grid_origin_.y) / resolution)),
      };
    }

    toy_rover::control::Point2D grid_to_world(toy_rover::mapping::GridIndex index) const
    {
      const double resolution = grid_.resolution_m();
      return toy_rover::control::Point2D{
          (static_cast<double>(index.x) + 0.5) * resolution + grid_origin_.x,
          (static_cast<double>(index.y) + 0.5) * resolution + grid_origin_.y,
      };
    }

    nav_msgs::msg::Path make_path_message(const std::vector<toy_rover::control::Point2D> &world_path)
    {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now();
      path_msg.header.frame_id = "map";
      path_msg.poses.reserve(world_path.size());

      for (const auto &point : world_path)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0; // we are not interested in that rn - we just want to pass the coords
        path_msg.poses.push_back(pose);
      }

      return path_msg;
    }

    double obstacle_inflation_radius_m_;
    double escape_inflation_radius_m_;
    double goal_snap_radius_m_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_publisher_;
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    std::optional<toy_rover::control::Point2D> adjusted_goal_;
    std::optional<std::chrono::steady_clock::time_point> last_recovery_request_;
    bool has_map_{false};
    toy_rover::control::Point2D grid_origin_{-20.0, -20.0};
    toy_rover::mapping::OccupancyGrid grid_{800, 800, 0.05};
    toy_rover::mapping::OccupancyGrid escape_grid_{800, 800, 0.05};
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
