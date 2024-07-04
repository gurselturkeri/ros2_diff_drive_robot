#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber() : Node("map_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MapSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received map with width: %d, height: %d", msg->info.width, msg->info.height);
    // Additional processing of the occupancy grid can be done here
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSubscriber>());
  rclcpp::shutdown();
  return 0;
}
