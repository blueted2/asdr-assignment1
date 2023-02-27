#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

using std::placeholders::_1;

class ShowLightPosition : public rclcpp::Node
{
public:
  ShowLightPosition()
  : Node("show_light_position")
  {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&ShowLightPosition::image_callback, this, _1));

    pos_in_pix_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position_pixels", 10, std::bind(&ShowLightPosition::position_callback, this, _1));

    img_with_pos_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_light_pos", 10);
  }

private:

  // when a new image is received, save it
  void image_callback(const sensor_msgs::msg::Image & image)
  {
    last_image_ = image;
    publish_image();
  }

  // when a new position is received, save it
  void position_callback(const asdfr_interfaces::msg::Point2 & position)
  {
    last_position_ = position;
    publish_image();
  }

  // when a new image or position is received, draw the position on the image and publish it
  void publish_image() {
    // convert the image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // draw the position on the image
    cv::circle(cv_ptr->image, cv::Point(last_position_.x, last_position_.y), 10, cv::Scalar(0, 0, 255), 2);

    // convert the image back to ROS format
    cv_ptr->encoding = "bgr8";
    sensor_msgs::msg::Image img_with_pos = *cv_ptr->toImageMsg();

    // publish the image
    img_with_pos_pub_->publish(img_with_pos);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr pos_in_pix_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_with_pos_pub_;

  // save the last image and position
  sensor_msgs::msg::Image last_image_;
  asdfr_interfaces::msg::Point2 last_position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShowLightPosition>());
  rclcpp::shutdown();
  return 0;
}
