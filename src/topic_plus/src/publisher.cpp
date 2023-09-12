#include <string>
#include <functional>
#include <chrono>
#include <cstdint>

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "tutorial_interfaces/msg/num.hpp"

namespace rcl = rclcpp;
using namespace std::chrono_literals;
using Interface = tutorial_interfaces::msg::Num;

class PublisherPlus: public rcl::Node {
public:
  PublisherPlus(const std::string& name)
    : rcl::Node(name)
  {
    publisher = this->create_publisher<Interface>("plus", 10);
    timer = this->create_wall_timer(500ms, std::bind(&PublisherPlus::timer_callback, this));
  }

  void timer_callback()
  {
    auto msg = Interface();
    msg.num = cnt++;
    RCLCPP_INFO(this->get_logger(), "publishing: %ld", msg.num);
    publisher->publish(msg);
  }

private:
  rcl::TimerBase::SharedPtr timer;
  rcl::Publisher<Interface>::SharedPtr publisher;
  int64_t cnt{};
};

int main(int argc, char* argv[])
{
  rcl::init(argc, argv);
  rcl::spin(std::make_shared<PublisherPlus>("publisherplus"));
  rcl::shutdown();
}
