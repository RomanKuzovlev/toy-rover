#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include "control/pose_history.hpp"
#include "control/pure_pursuit.hpp"
#include "control/stuck_detector.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mapping/obstacle_history.hpp"
#include "mapping/occupancy_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"

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
          grid_(280, 280, 0.1),
          pose_history_(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(
                  declare_parameter<double>("odometry_history_seconds", 2.0)))),
          obstacle_history_(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(
                  declare_parameter<double>("obstacle_retention_seconds", 3.0)))),
          stuck_detector_(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::duration<double>(
                      declare_parameter<double>("stuck_detection_window_seconds", 2.0))),
              declare_parameter<int>("stuck_cell_tolerance", 1)),
          enable_stuck_recovery_(declare_parameter<bool>("enable_stuck_recovery", false)),
          lidar_offset_x_m_(declare_parameter<double>("lidar_offset_x_m", 0.08)),
          lidar_offset_y_m_(declare_parameter<double>("lidar_offset_y_m", 0.0)),
          grid_pub_(create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10)),
          unblock_pub_(create_publisher<std_msgs::msg::Empty>("unblock", 10))
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
      if (pending_scans_.size() == max_pending_scans_)
      {
        pending_scans_.pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Odometry did not catch up with scans; dropping oldest pending scan");
      }
      pending_scans_.push_back(msg);
      process_pending_scans();
    }

    void process_scan(
        const sensor_msgs::msg::LaserScan &msg,
        const toy_rover::control::Pose2D &base_pose)
    {
      const double cos_yaw = std::cos(base_pose.yaw_rad);
      const double sin_yaw = std::sin(base_pose.yaw_rad);
      const toy_rover::control::Point2D lidar_origin{
          base_pose.x + cos_yaw * lidar_offset_x_m_ - sin_yaw * lidar_offset_y_m_,
          base_pose.y + sin_yaw * lidar_offset_x_m_ + cos_yaw * lidar_offset_y_m_,
      };

      std::vector<toy_rover::mapping::GridIndex> occupied_cells;
      occupied_cells.reserve(msg.ranges.size());
      for (std::size_t i = 0; i < msg.ranges.size(); ++i)
      {
        const float range = msg.ranges[i];
        if (!std::isfinite(range) || range < msg.range_min || range > msg.range_max)
        {
          continue;
        }

        const double scan_angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
        const double world_angle = base_pose.yaw_rad + scan_angle;
        const toy_rover::control::Point2D hit{
            lidar_origin.x + std::cos(world_angle) * range,
            lidar_origin.y + std::sin(world_angle) * range,
        };

        const auto cell = world_to_grid(hit);
        if (grid_.in_bounds(cell))
        {
          occupied_cells.push_back(cell);
        }
      }

      obstacle_history_.update(
          std::chrono::nanoseconds{rclcpp::Time(msg.header.stamp).nanoseconds()},
          std::move(occupied_cells),
          grid_);
      grid_pub_->publish(make_grid_message());
    }

    void process_pending_scans()
    {
      while (!pending_scans_.empty())
      {
        const auto &scan = pending_scans_.front();
        const auto scan_timestamp =
            std::chrono::nanoseconds{rclcpp::Time(scan.header.stamp).nanoseconds()};
        const auto pose = pose_history_.interpolate(scan_timestamp);
        if (pose)
        {
          process_scan(scan, *pose);
          pending_scans_.pop_front();
          continue;
        }

        const auto latest_odom = pose_history_.latest_timestamp();
        if (!latest_odom || scan_timestamp > *latest_odom)
        {
          return;
        }

        // The scan predates retained odometry and can never be interpolated.
        pending_scans_.pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping scan older than retained odometry history");
      }
    }

    void on_odometry(const nav_msgs::msg::Odometry &msg)
    {
      const toy_rover::control::Pose2D pose{
          msg.pose.pose.position.x,
          msg.pose.pose.position.y,
          yaw_from_quaternion(msg.pose.pose.orientation),
      };
      const auto timestamp =
          std::chrono::nanoseconds{rclcpp::Time(msg.header.stamp).nanoseconds()};
      const auto previous_timestamp = pose_history_.latest_timestamp();
      if (previous_timestamp && timestamp < *previous_timestamp)
      {
        pending_scans_.clear();
      }
      pose_history_.add(timestamp, pose);

      const auto cell = world_to_grid({pose.x, pose.y});
      if (enable_stuck_recovery_ && stuck_detector_.update(timestamp, cell))
      {
        unblock_pub_->publish(std_msgs::msg::Empty{});
        RCLCPP_WARN(get_logger(), "Rover appears stuck; publishing unblock event");
      }

      process_pending_scans();
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
    toy_rover::control::PoseHistory pose_history_;
    toy_rover::mapping::ObstacleHistory obstacle_history_;
    toy_rover::control::StuckDetector stuck_detector_;
    bool enable_stuck_recovery_;
    toy_rover::control::Point2D grid_origin_{-14.0, -14.0};
    double lidar_offset_x_m_;
    double lidar_offset_y_m_;
    static constexpr std::size_t max_pending_scans_{20};
    std::deque<sensor_msgs::msg::LaserScan> pending_scans_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr unblock_pub_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
