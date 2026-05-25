#include <memory>

#include "planning/astar.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  class PlannerNode : public rclcpp::Node
  {
  public:
    PlannerNode() : Node("planner_node")
    {
      // subscribe to a goal
      // issue a timer
      // update the goal
      // if the goal is changed, switch some bool to retrigger path construction
      // construct path with using astar
      // send path to controller node once to execute
      RCLCPP_INFO(get_logger(), "planner node started");
    }

  private:
    toy_rover::planning::AStar planner_;
  };
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
