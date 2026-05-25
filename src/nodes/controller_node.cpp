#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "control/pure_pursuit.hpp"
#include "control/velocity_limiter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

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
          path_{{1.0, 0.0}, {2.0, 0.0}},
          cmd_pub_(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10))
    {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
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
        publish_stop();
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

    void publish_stop()
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    toy_rover::control::PurePursuit tracker_;
    toy_rover::control::VelocityLimiter limiter_;
    std::vector<toy_rover::control::Point2D> path_;
    std::optional<toy_rover::control::Pose2D> latest_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
