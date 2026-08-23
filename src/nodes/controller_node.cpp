#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "control/free_space_escape.hpp"
#include "control/pure_pursuit.hpp"
#include "control/velocity_limiter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

namespace
{
  double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  class ControllerNode : public rclcpp::Node
  {
  public:
    ControllerNode()
        : Node("controller_node"),
          tracker_(
              declare_parameter<double>("lookahead_m", 0.45),
              declare_parameter<double>("cruise_speed_mps", 0.35)),
          limiter_(
              declare_parameter<double>("max_linear_speed_mps", 0.55),
              declare_parameter<double>("max_angular_speed_radps", 1.8)),
          recovery_speed_mps_(declare_parameter<double>("recovery_speed_mps", 0.22)),
          recovery_drive_duration_(declare_parameter<double>(
              "recovery_drive_duration_seconds", 1.5)),
          recovery_stop_clearance_m_(declare_parameter<double>(
              "recovery_stop_clearance_m", 0.45)),
          recovery_corridor_half_angle_rad_(declare_parameter<double>(
              "recovery_corridor_half_angle_rad", 0.25)),
          cmd_pub_(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10))
    {
      if (recovery_speed_mps_ <= 0.0 || recovery_drive_duration_.count() <= 0.0 ||
          recovery_stop_clearance_m_ <= 0.0 || recovery_corridor_half_angle_rad_ <= 0.0)
      {
        throw std::invalid_argument("recovery parameters must be positive");
      }

      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
          });
      path_sub_ = create_subscription<nav_msgs::msg::Path>(
          "planned_path",
          10,
          [this](const nav_msgs::msg::Path::SharedPtr msg)
          {
            on_path(*msg);
          });
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          10,
          [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
          {
            latest_scan_ = *msg;
          });
      unblock_sub_ = create_subscription<std_msgs::msg::Empty>(
          "unblock",
          10,
          [this](const std_msgs::msg::Empty::SharedPtr)
          {
            on_unblock();
          });
      timer_ = create_wall_timer(50ms, [this]
                                 { on_timer(); });
      RCLCPP_INFO(get_logger(), "controller node started");
    }

  private:
    void on_timer()
    {
      if (!latest_pose_)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "No odometry received yet; publishing stop command");
        publish_stop();
        return;
      }

      if (run_recovery())
      {
        return;
      }

      const auto limited = limiter_.limit(tracker_.compute_command(*latest_pose_, path_));
      geometry_msgs::msg::Twist msg;
      msg.linear.x = limited.linear_mps;
      msg.angular.z = limited.angular_radps;
      cmd_pub_->publish(msg);
    }

    void on_odometry(const nav_msgs::msg::Odometry &msg)
    {
      latest_pose_ = toy_rover::control::Pose2D{
          msg.pose.pose.position.x,
          msg.pose.pose.position.y,
          yaw_from_quaternion(msg.pose.pose.orientation),
      };
    }

    void on_path(const nav_msgs::msg::Path &msg)
    {
      path_.clear();
      path_.reserve(msg.poses.size());

      for (const auto &pose_stamped : msg.poses)
      {
        path_.push_back(toy_rover::control::Point2D{
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
        });
      }
    }

    void on_unblock()
    {
      if (recovery_phase_ != RecoveryPhase::Idle)
      {
        return;
      }
      if (!latest_pose_ || !latest_scan_)
      {
        RCLCPP_WARN(get_logger(), "Cannot start local escape without odometry and lidar");
        return;
      }

      const auto relative_heading = toy_rover::control::choose_free_space_heading(
          latest_scan_->ranges,
          latest_scan_->angle_min,
          latest_scan_->angle_increment,
          latest_scan_->range_min,
          latest_scan_->range_max,
          recovery_corridor_half_angle_rad_);
      if (!relative_heading)
      {
        RCLCPP_WARN(get_logger(), "Cannot find a usable lidar direction for local escape");
        return;
      }

      recovery_target_yaw_rad_ = normalize_angle(
          latest_pose_->yaw_rad + *relative_heading);
      recovery_phase_ = RecoveryPhase::Turning;
      RCLCPP_WARN(
          get_logger(), "Local escape selected lidar heading %.2f rad", *relative_heading);
    }

    bool run_recovery()
    {
      if (recovery_phase_ == RecoveryPhase::Idle)
      {
        return false;
      }

      if (recovery_phase_ == RecoveryPhase::Turning)
      {
        const double error = normalize_angle(recovery_target_yaw_rad_ - latest_pose_->yaw_rad);
        constexpr double heading_tolerance_rad = 0.12;
        if (std::abs(error) > heading_tolerance_rad)
        {
          geometry_msgs::msg::Twist command;
          command.angular.z = std::clamp(2.0 * error, -1.2, 1.2);
          cmd_pub_->publish(command);
          return true;
        }
        recovery_phase_ = RecoveryPhase::Driving;
        recovery_drive_started_at_ = std::chrono::steady_clock::now();
        publish_stop();
        return true;
      }

      const double clearance = latest_scan_
                                   ? toy_rover::control::forward_clearance(
                                         latest_scan_->ranges,
                                         latest_scan_->angle_min,
                                         latest_scan_->angle_increment,
                                         latest_scan_->range_min,
                                         latest_scan_->range_max,
                                         recovery_corridor_half_angle_rad_)
                                   : 0.0;
      const auto elapsed = std::chrono::steady_clock::now() - *recovery_drive_started_at_;
      if (elapsed >= recovery_drive_duration_ ||
          clearance < recovery_stop_clearance_m_)
      {
        recovery_phase_ = RecoveryPhase::Idle;
        recovery_drive_started_at_.reset();
        publish_stop();
        RCLCPP_INFO(
            get_logger(), "Local escape finished (forward clearance %.2f m)", clearance);
        return true;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = recovery_speed_mps_;
      cmd_pub_->publish(command);
      return true;
    }

    static double normalize_angle(double angle)
    {
      return std::atan2(std::sin(angle), std::cos(angle));
    }

    void publish_stop()
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    toy_rover::control::PurePursuit tracker_;
    toy_rover::control::VelocityLimiter limiter_;
    enum class RecoveryPhase
    {
      Idle,
      Turning,
      Driving,
    };
    double recovery_speed_mps_;
    std::chrono::duration<double> recovery_drive_duration_;
    double recovery_stop_clearance_m_;
    double recovery_corridor_half_angle_rad_;
    std::vector<toy_rover::control::Point2D> path_;
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    std::optional<sensor_msgs::msg::LaserScan> latest_scan_;
    RecoveryPhase recovery_phase_{RecoveryPhase::Idle};
    double recovery_target_yaw_rad_{0.0};
    std::optional<std::chrono::steady_clock::time_point> recovery_drive_started_at_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr unblock_sub_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
