#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/highgui/highgui.hpp"

using std::placeholders::_1;

class LightPosition : public rclcpp::Node
{
public:
  LightPosition()
  : Node("light_position")
  {
    this->declare_parameter<double>("threshold", 70.0f);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&LightPosition::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("light_position", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image & image) const
  {
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

    auto message = geometry_msgs::msg::Point();

    message.x = center_of_mass.x;
    message.y = center_of_mass.y;

    publisher_->publish(message);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
