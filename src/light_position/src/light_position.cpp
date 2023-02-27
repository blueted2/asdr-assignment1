#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "asdfr_interfaces/msg/point2.hpp"

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

    pos_in_pix_pub_     = this->create_publisher<asdfr_interfaces::msg::Point2>("position_pixels", 10);
    pos_norm_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("position_normalized", 10);
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

    auto message_pixels = asdfr_interfaces::msg::Point2();
    auto message_normalized = asdfr_interfaces::msg::Point2();

    message_pixels.x = center_of_mass.x;
    message_pixels.y = center_of_mass.y;

    message_normalized.x = (center_of_mass.x / (double)threshold_img.cols) * 2 - 1;
    message_normalized.y = (center_of_mass.y / (double)threshold_img.rows) * 2 - 1;

    // log both the pixel and normalized coordinates
    RCLCPP_INFO(this->get_logger(), "cog in pixels: (%d, %d); cog normalized: (%.2f, %.2f)", center_of_mass.x, center_of_mass.y, message_normalized.x, message_normalized.y);

    pos_in_pix_pub_->publish(message_pixels);
    pos_norm_pub_->publish(message_normalized);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr pos_in_pix_pub_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr pos_norm_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
