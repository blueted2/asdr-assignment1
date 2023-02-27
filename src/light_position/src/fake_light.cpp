// node that publishes an image of a white circle on a black background which moves around the screen

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "asdfr_interfaces/msg/point2.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class FakeLight : public rclcpp::Node
{
public:
  FakeLight()
  : Node("fake_light")
  {
    auto timer_callback = std::bind(&FakeLight::image_callback, this);
    auto period = std::chrono::milliseconds(33);
    timer_ = this->create_wall_timer(period, timer_callback);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  }

private:
  
  void image_callback()
  {
    static int i = 0;
    if (i++ > 100) {

      // pick two random numbers between -0.1 and 0.1
      float x = 0.2f * ((float)rand() / RAND_MAX - 0.5f);
      float y = 0.2f * ((float)rand() / RAND_MAX - 0.5f);
      
      // move the light in a random direction
      light_position.x += x;
      light_position.y += y;

      // make sure the light stays within the image
      if (light_position.x < 0.0f) light_position.x = 0.0f;
      if (light_position.x > 1.0f) light_position.x = 1.0f;
      if (light_position.y < 0.0f) light_position.y = 0.0f;
      if (light_position.y > 1.0f) light_position.y = 1.0f;

      i = 0;
    }


    // create image
    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(127, 0, 0));

    // draw a sqaure at each corner of the image
    cv::rectangle(image, cv::Point(0, 0), cv::Point(50, 50), cv::Scalar(0, 0, 127), -1);
    cv::rectangle(image, cv::Point(image_width - 50, 0), cv::Point(image_width, 50), cv::Scalar(0, 0, 127), -1);
    cv::rectangle(image, cv::Point(0, image_height - 50), cv::Point(50, image_height), cv::Scalar(0, 0, 127), -1);
    cv::rectangle(image, cv::Point(image_width - 50, image_height - 50), cv::Point(image_width, image_height), cv::Scalar(0, 0, 127), -1);


    cv::circle(image, cv::Point(light_position.x * image_width, light_position.y * image_height), 50, cv::Scalar(255, 255, 255), -1);

    // convert image to ROS message
    auto msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    // publish image
    image_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const int image_width = 640;
  const int image_height = 480;

  // light position
  asdfr_interfaces::msg::Point2 light_position = asdfr_interfaces::msg::Point2().set__x(0.5f).set__y(0.5f);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLight>());
  rclcpp::shutdown();
  return 0;
}
