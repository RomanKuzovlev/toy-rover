#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
  class OdomTfBroadcasterNode : public rclcpp::Node
  {
  public:
    OdomTfBroadcasterNode()
        : Node("odom_tf_broadcaster"),
          broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
    {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom",
          10,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            on_odometry(*msg);
          });

      RCLCPP_INFO(get_logger(), "odom tf broadcaster started");
    }

  private:
    void on_odometry(const nav_msgs::msg::Odometry &msg)
    {
      const std::string child_frame =
          msg.child_frame_id.empty() ? "base_footprint" : msg.child_frame_id;

      geometry_msgs::msg::TransformStamped transform;
      transform.header = msg.header;
      transform.child_frame_id = child_frame;
      transform.transform.translation.x = msg.pose.pose.position.x;
      transform.transform.translation.y = msg.pose.pose.position.y;
      transform.transform.translation.z = msg.pose.pose.position.z;
      transform.transform.rotation = msg.pose.pose.orientation;

      broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
