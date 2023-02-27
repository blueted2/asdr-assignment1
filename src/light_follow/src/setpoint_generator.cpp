#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class SetpointGenerator : public rclcpp::Node
{
public:
  SetpointGenerator()
  : Node("setpoint_generator")
  {
    setpoint_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);

    // publish a setpoint every 1 second
    auto timer_callback = std::bind(&SetpointGenerator::setpoint_callback, this);
    auto period = std::chrono::milliseconds(1000);
    timer_ = this->create_wall_timer(period, timer_callback);
  }

private:
  
  void setpoint_callback()
  {
    auto setpoint = setpoints[setpoint_index];
    setpoint_index = (setpoint_index + 1) % setpoints.size();

    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: (%f, %f)", setpoint.x, setpoint.y);

    setpoint_pub_->publish(setpoint);
  }

  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_pub_;

  // constant list of setpoints
  const std::vector<asdfr_interfaces::msg::Point2> setpoints = {
    asdfr_interfaces::msg::Point2().set__x(0.0f).set__y(0.0f),
    asdfr_interfaces::msg::Point2().set__x(0.5f).set__y(0.0f),
    asdfr_interfaces::msg::Point2().set__x(0.5f).set__y(0.5f),
    asdfr_interfaces::msg::Point2().set__x(0.0f).set__y(0.5f),
  };

  // current setpoint index
  int setpoint_index = 0;

  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointGenerator>());
  rclcpp::shutdown();
  return 0;
}
