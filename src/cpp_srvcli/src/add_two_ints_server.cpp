#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <functional>
#include <memory>

using tutorial_interfaces::srv::AddThreeInts;
using namespace std::placeholders;

static const std::string service_name = "add_three_ints";

class AddServer: public rclcpp::Node {
public:
  AddServer(const std::string& name)
    : rclcpp::Node(name)
  {
    service = this->create_service<AddThreeInts>(service_name, std::bind(&AddServer::add, this, _1, _2));
  }

  void add(const std::shared_ptr<AddThreeInts::Request> request,
            std::shared_ptr<AddThreeInts::Response>      response)
  {
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(this->get_logger(), "incoming request\n" "a: %ld" " b: %ld" " c: %ld" ,
                  request->a, request->b, request->c);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]", (long int)response->sum);
  }

private:
  rclcpp::Service<AddThreeInts>::SharedPtr service;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AddServer>("add_three_ints_server");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
