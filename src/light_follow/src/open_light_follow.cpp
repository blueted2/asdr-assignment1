#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class ClosedLightFollower : public rclcpp::Node
{
public:
  ClosedLightFollower()
  : Node("norm2rad")
  {
    light_pos_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position_normalized", 10, std::bind(&ClosedLightFollower::topic_callback, this, _1));

    setpoint_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
  }

private:
  void topic_callback(const asdfr_interfaces::msg::Point2 & point) const
  {
    const float x_rads_max = 0.8;
    const float y_rads_max = 0.6;

    float x_rads = point.x * x_rads_max;
    float y_rads = -point.y * y_rads_max;

    auto message = asdfr_interfaces::msg::Point2();
    
    message.x = x_rads;
    message.y = y_rads;

    setpoint_pub_->publish(message);
  }
  
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr light_pos_sub_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLightFollower>());
  rclcpp::shutdown();
  return 0;
}
