#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

using std::placeholders::_1;

class ShowJiwyPos : public rclcpp::Node
{
public:
  ShowJiwyPos()
  : Node("show_light_position")
  {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&ShowJiwyPos::image_callback, this, _1));

    pos_in_rads_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position", 10, std::bind(&ShowJiwyPos::position_callback, this, _1));

    img_with_pos_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_jiwy_pos", 10);
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
    pos_in_rads_ = position;
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

    const double pixels_per_rad_x = 320 / 0.8;
    const double pixels_per_rad_y = 240 / 0.6;

    // image has coordinates (-0.8, 0.8) in x and (-0.6, 0.6) in y
    // convert to pixel coordinates
    int x = (pos_in_rads_.x - 0.4) * pixels_per_rad_x + 320;
    int y = (-pos_in_rads_.y - 0.3) * pixels_per_rad_y + 240;

    int width = 0.8 * pixels_per_rad_x;
    int height = 0.6 * pixels_per_rad_y;

    cv::rectangle(cv_ptr->image, cv::Point(x, y), cv::Point(x + width, y + height), cv::Scalar(0, 0, 255), 2);

    // convert the image back to ROS format
    cv_ptr->encoding = "bgr8";
    sensor_msgs::msg::Image img_with_pos = *cv_ptr->toImageMsg();

    // publish the image
    img_with_pos_pub_->publish(img_with_pos);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr pos_in_rads_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_with_pos_pub_;

  // save the last image and position
  sensor_msgs::msg::Image last_image_;
  asdfr_interfaces::msg::Point2 pos_in_rads_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShowJiwyPos>());
  rclcpp::shutdown();
  return 0;
}
