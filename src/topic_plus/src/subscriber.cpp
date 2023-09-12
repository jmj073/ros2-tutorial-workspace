#include <string>
#include <functional>

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "tutorial_interfaces/msg/num.hpp"

namespace rcl = rclcpp;
using namespace std::chrono_literals;
using Interface = tutorial_interfaces::msg::Num;
using namespace std::placeholders;

class SubscriberPlus: public rcl::Node {
public:
  SubscriberPlus(const std::string& name)
    : Node(name)
  {
    subscription = this->create_subscription<Interface>("plus", 10, std::bind(&SubscriberPlus::sub_callback, this, _1));
  }

private:
  void sub_callback(const Interface& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: %ld", msg.num);
  }

private:
  rcl::Subscription<Interface>::SharedPtr subscription;
};

int main(int argc, char* argv[])
{
  rcl::init(argc, argv);
  rcl::spin(std::make_shared<SubscriberPlus>("subscriberplus"));
  rcl::shutdown();
}
