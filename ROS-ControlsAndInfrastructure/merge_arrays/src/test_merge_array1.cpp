#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TestArray1 : public rclcpp::Node
{
  public:
    TestArray1() : Node("test_array1"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/input/array1", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TestArray1::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Int32MultiArray();
      message.data = {3, 6, 13, 19, 29};
      for (int i{ 0 }; i < size(message.data); i++) {
    	RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", message.data[i]);
      }
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestArray1>());
  rclcpp::shutdown();
  return 0;
}
