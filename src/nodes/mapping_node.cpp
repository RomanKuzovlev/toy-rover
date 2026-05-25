#include <memory>

#include "mapping/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  class MappingNode : public rclcpp::Node
  {
  public:
    MappingNode() : Node("mapping_node"), grid_(80, 80, 0.1)
    {
      RCLCPP_INFO(get_logger(), "mapping node started: %dx%d grid", grid_.width(), grid_.height());
    }

  private:
    toy_rover::mapping::OccupancyGrid grid_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
