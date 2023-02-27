#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class BrightnessNode : public rclcpp::Node
{
public:
  BrightnessNode()
  : Node("brightness_node")
  {

    this->declare_parameter("threshold", 5);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&BrightnessNode::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Bool>("light_state", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image & image) const
  {
    // convert image to opencv format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

    // convert to grayscale
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // calculate mean light level
    cv::Scalar mean = cv::mean(gray);

    // get threshold from parameter
    int th = this->get_parameter("threshold").get_parameter_value().get<int>();

    // as this image is grayscale, only one channel is used
    double light_level = mean[0];

    // compare light level to threshold
    bool light_state = light_level > th;

    auto message = std_msgs::msg::Bool();
    message.data = light_state;

    RCLCPP_INFO(this->get_logger(), "mean light level: %f, %s", mean[0], light_state ? "on" : "off");
    
    publisher_->publish(message);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrightnessNode>());
  rclcpp::shutdown();
  return 0;
}
