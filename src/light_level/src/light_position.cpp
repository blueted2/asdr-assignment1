#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class LightPosition : public rclcpp::Node
{
public:
  LightPosition()
  : Node("light_position")
  {
    this->declare_parameter<double>("threshold", 100.0f);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "moving_camera_output", 10, std::bind(&LightPosition::topic_callback, this, _1));

    publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("light_position", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image & image) const
  {
    float height=240, width=320;
    // convert image to opencv format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    // convert to grayscale
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // get threshold from parameter
    double threshold = this->get_parameter("threshold").as_double();

    // apply threshold
    cv::Mat threshold_img;
    cv::threshold(gray, threshold_img, threshold, 255, cv::THRESH_BINARY);

    // calculate center of mass of light
    cv::Moments moments = cv::moments(threshold_img, true);
    cv::Point center_of_mass = cv::Point(moments.m10 / moments.m00, moments.m01 / moments.m00);

    auto message = asdfr_interfaces::msg::Point2();

    message.x = center_of_mass.x / width;
    message.y = center_of_mass.y / height;

    publisher_->publish(message);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
