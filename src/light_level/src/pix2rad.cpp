#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class Pix2Rad : public rclcpp::Node
{
public:
  Pix2Rad()
  : Node("pix2rad")
  {
    // this->declare_parameter<double>("threshold", 70.0f);

    subscription_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "pixels", 10, std::bind(&Pix2Rad::topic_callback, this, _1));

    publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("radians", 10);
  }

private:
  void topic_callback(const asdfr_interfaces::msg::Point2 & point) const
  {
    const int img_width = 640;
    const int img_height = 480;

    const float x_rads_max = 0.8;
    const float y_rads_max = 0.6;

    float x_rads = ((point.x / img_width) - 0.5) * 2 * x_rads_max;
    float y_rads = -((point.y / img_height) - 0.5) * 2 * y_rads_max;

    auto message = asdfr_interfaces::msg::Point2();
    
    message.x = x_rads;
    message.y = y_rads;

    publisher_->publish(message);
  }
  
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr subscription_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pix2Rad>());
  rclcpp::shutdown();
  return 0;
}
