#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

    // Extract image dimensions
    int width = image.width;
    int height = image.height;

    // Extract pixel data
    const uint8_t* data = image.data.data();

    // Calculate total mass and COG
    double totalMass = 0.0;
    double xCog = 0.0;
    double yCog = 0.0;

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {

        int index = (y * width + x) * 3;

        // Calculate intensity
        double intensity = (data[index] + data[index + 1] + data[index + 2]);


        if (intensity < this->get_parameter("threshold").as_double()) {
          continue;
        }

        // Update total mass and COG
        totalMass += intensity;
        xCog += x * intensity;
        yCog += y * intensity;
      }
    }

    xCog /= totalMass;
    yCog /= totalMass;


    auto message_pixels = asdfr_interfaces::msg::Point2();
    auto message_normalized = asdfr_interfaces::msg::Point2();

    message_pixels.x = xCog;
    message_pixels.y = yCog;

    message_normalized.x = (xCog / (double)width) * 2 - 1;
    message_normalized.y = (yCog / (double)height) * 2 - 1;

    // log both the pixel and normalized coordinates
    RCLCPP_INFO(this->get_logger(), "cog in pixels: (%.2f, %.2f); cog normalized: (%.2f, %.2f)", xCog, yCog, message_normalized.x, message_normalized.y);

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
