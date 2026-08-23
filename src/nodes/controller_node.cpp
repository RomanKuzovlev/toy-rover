#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "control/pure_pursuit.hpp"
#include "control/velocity_limiter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
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
          tracker_(0.3, 0.15),
          limiter_(0.35, 1.2),
          reverse_speed_mps_(declare_parameter<double>("unblock_reverse_speed_mps", 0.15)),
          reverse_duration_(rclcpp::Duration::from_seconds(
              declare_parameter<double>("unblock_reverse_duration_seconds", 1.0))),
          cmd_pub_(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10))
    {
      if (reverse_speed_mps_ <= 0.0 || reverse_duration_.nanoseconds() <= 0)
      {
        throw std::invalid_argument("unblock reverse speed and duration must be positive");
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
        RCLCPP_WARN(get_logger(), "No odometry received yet; publishing stop command");
        // publish_stop();
        return;
      }

      if (reverse_started_at_)
      {
        const auto elapsed = now() - *reverse_started_at_;
        if (elapsed >= rclcpp::Duration::from_nanoseconds(0) && elapsed < reverse_duration_)
        {
          geometry_msgs::msg::Twist reverse;
          reverse.linear.x = -reverse_speed_mps_;
          cmd_pub_->publish(reverse);
          return;
        }
        reverse_started_at_.reset();
        RCLCPP_INFO(get_logger(), "Unblock reverse maneuver finished; resuming path tracking");
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
      if (reverse_started_at_)
      {
        return;
      }

      reverse_started_at_ = now();
      RCLCPP_WARN(get_logger(), "Unblock event received; reversing temporarily");
    }

    void publish_stop()
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    toy_rover::control::PurePursuit tracker_;
    toy_rover::control::VelocityLimiter limiter_;
    double reverse_speed_mps_;
    rclcpp::Duration reverse_duration_;
    std::vector<toy_rover::control::Point2D> path_;
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    std::optional<rclcpp::Time> reverse_started_at_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
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
