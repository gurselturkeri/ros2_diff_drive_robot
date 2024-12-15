#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;   // create_wall_timer için

class VelocityPublisher : public rclcpp::Node {
    public:
        VelocityPublisher():Node("velocity_publisher")
{
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&VelocityPublisher::publishVelocity, this));
}

    private:
    void publishVelocity(){
    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = 0.0;
    vel_publisher -> publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "Published velocity");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}