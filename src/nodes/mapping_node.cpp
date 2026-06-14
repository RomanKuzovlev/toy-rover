#include <cmath>
#include <memory>
#include <optional>

#include "control/pure_pursuit.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mapping/occupancy_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
  double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  }

  class MappingNode : public rclcpp::Node
  {
  public:
    MappingNode()
        : Node("mapping_node"),
          grid_(80, 80, 0.1),
          grid_pub_(create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10))
    {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          10,
          [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
          {
            on_scan(*msg);
          });

      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
          });

      RCLCPP_INFO(get_logger(), "mapping node started: %dx%d grid", grid_.width(), grid_.height());
    }

  private:
    void on_scan(const sensor_msgs::msg::LaserScan &msg)
    {
      if (!latest_pose_)
      {
        return;
      }

      bool changed = false;
      for (std::size_t i = 0; i < msg.ranges.size(); ++i)
      {
        const float range = msg.ranges[i];
        if (!std::isfinite(range) || range < msg.range_min || range > msg.range_max)
        {
          continue;
        }

        const double scan_angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
        const double world_angle = latest_pose_->yaw_rad + scan_angle;
        const toy_rover::control::Point2D hit{
            latest_pose_->x + std::cos(world_angle) * range,
            latest_pose_->y + std::sin(world_angle) * range,
        };

        const auto cell = world_to_grid(hit);
        if (grid_.in_bounds(cell))
        {
          grid_.set(cell, toy_rover::mapping::Cell::Occupied);
          changed = true;
        }
      }

      if (changed)
      {
        grid_pub_->publish(make_grid_message());
      }
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
          static_cast<int>(std::floor((point.x - grid_origin_.x) / resolution)),
          static_cast<int>(std::floor((point.y - grid_origin_.y) / resolution)),
      };
    }

    nav_msgs::msg::OccupancyGrid make_grid_message()
    {
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";
      msg.info.resolution = static_cast<float>(grid_.resolution_m());
      msg.info.width = static_cast<std::uint32_t>(grid_.width());
      msg.info.height = static_cast<std::uint32_t>(grid_.height());
      msg.info.origin.position.x = grid_origin_.x;
      msg.info.origin.position.y = grid_origin_.y;
      msg.info.origin.orientation.w = 1.0;
      msg.data.reserve(grid_.cells().size());
      for (const auto cell : grid_.cells())
      {
        msg.data.push_back(static_cast<std::int8_t>(cell));
      }
      return msg;
    }

    toy_rover::mapping::OccupancyGrid grid_;
    toy_rover::control::Point2D grid_origin_{-4.0, -4.0};
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
