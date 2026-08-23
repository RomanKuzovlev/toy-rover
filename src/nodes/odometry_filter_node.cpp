#include <chrono>
#include <memory>

#include "control/timestamp_gate.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  class OdometryFilterNode : public rclcpp::Node
  {
  public:
    OdometryFilterNode()
        : Node("odometry_filter"),
          odom_pub_(create_publisher<nav_msgs::msg::Odometry>("odom", 10))
    {
      raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "raw_odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            const auto message_timestamp =
                std::chrono::nanoseconds{rclcpp::Time(msg->header.stamp).nanoseconds()};
            const auto clock_timestamp =
                std::chrono::nanoseconds{now().nanoseconds()};
            if (!timestamp_gate_.accept(message_timestamp, clock_timestamp))
            {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "Dropping out-of-order raw odometry sample");
              return;
            }
            odom_pub_->publish(*msg);
          });

      RCLCPP_INFO(get_logger(), "odometry timestamp filter started");
    }

  private:
    toy_rover::control::TimestampGate timestamp_gate_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryFilterNode>());
  rclcpp::shutdown();
  return 0;
}
