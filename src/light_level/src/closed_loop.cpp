#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class Closed_Loop : public rclcpp::Node
{
public:
  Closed_Loop()
  : Node("Closed_Loop")
  {
    // this->declare_parameter<double>("threshold", 70.0f);

    subscription_to_LP = this->create_subscription<asdfr_interfaces::msg::Point2>(
    "light_position", 10, std::bind(&Closed_Loop::get_light_position, this, _1));

    //position is set in radians
    subscription_to_position = this->create_subscription<asdfr_interfaces::msg::Point2>(
    "position", 10, std::bind(&Closed_Loop::feedback, this, _1));

    publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
  }

private:
  void get_light_position(const asdfr_interfaces::msg::Point2 & LP) 
  {
    light_position.x = LP.x;
    light_position.y = LP.y;
  }
  void feedback(const asdfr_interfaces::msg::Point2 & pos_moving_cam) 
  {
    float b = 15;
    float x_rads;
    float y_rads;
    x_rads = b * (light_position.x * 2 - 0.5) + pos_moving_cam.x;
    y_rads = - b * (light_position.y * 2 - 0.5) + pos_moving_cam.y;
    RCLCPP_INFO(this->get_logger(), "\tReceived setpoint: [%f, %f]\n\tposition moving camera: [%f %f]\n\tposition light position: [%f %f]", x_rads, y_rads, pos_moving_cam.x, pos_moving_cam.y, pos_moving_cam.x, pos_moving_cam.y);

    auto message = asdfr_interfaces::msg::Point2();
    
    message.x = x_rads;
    message.y = y_rads;

    publisher_->publish(message);
  }
  asdfr_interfaces::msg::Point2 light_position;
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr subscription_to_LP;
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr subscription_to_position;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Closed_Loop>());
  rclcpp::shutdown();
  return 0;
}
