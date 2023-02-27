#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;

class ClosedFollower : public rclcpp::Node
{
public:
  ClosedFollower()
  : Node("closed_follower"),
    time_step_(0.01)
  {
    light_pos_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "light_position_normalized", 10, std::bind(&ClosedFollower::lightpos_callback, this, _1));

    new_setpoint_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);

    this->declare_parameter<double>("tau_s", 1.0);

    auto timer_callback = std::bind(&ClosedFollower::update_setpoint, this);
    auto period = std::chrono::duration<double>(time_step_);
    timer_ = this->create_wall_timer(period, timer_callback);
  }

private:
  void lightpos_callback(const asdfr_interfaces::msg::Point2 & point)
  {
    light_position = point;
  }

  void update_setpoint() {
    double tau = this->get_parameter("tau_s").as_double();

    // full image maps x to [-0.8, 0.8] and y to [-0.6, 0.6]
    // but the jiwy only sees half of that
    const double x_radians_per_unit = 0.8 / 2;
    const double y_radians_per_unit = 0.6 / 2;

    // we already get the light position relative to the center of the image
    double relative_x = light_position.x * x_radians_per_unit;
    double relative_y = - light_position.y * y_radians_per_unit; // y is upside down

    // velocity
    double dx = relative_x / tau;
    double dy = relative_y / tau;

    // dv * dt
    jiwy_setpoint.x += dx * time_step_;
    jiwy_setpoint.y += dy * time_step_;

    auto message = asdfr_interfaces::msg::Point2();
    message.set__x(jiwy_setpoint.x).set__y(jiwy_setpoint.y);

    new_setpoint_pub_->publish(message);
  }
  
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr light_pos_sub_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr new_setpoint_pub_;

  asdfr_interfaces::msg::Point2 jiwy_setpoint;
  asdfr_interfaces::msg::Point2 light_position;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  double time_step_; // Approximate time step in seconds
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedFollower>());
  rclcpp::shutdown();
  return 0;
}
