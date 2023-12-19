#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>



void callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%f'", msg->wrench.force.x);
  
}