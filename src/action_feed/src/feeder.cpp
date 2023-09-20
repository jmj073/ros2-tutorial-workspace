// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <thread>
#include <map>

#include "action_feed/action/feed.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

class Feeder : public rclcpp::Node
{
public: // type definitions
  using Message = action_feed::action::Feed;
  using GoalHandleMessage = rclcpp_action::ServerGoalHandle<Message>;

public: // public member functions

  explicit Feeder (const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("feeder", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Message>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "feeder",
      std::bind(&Feeder::handle_goal, this, _1, _2),
      std::bind(&Feeder::handle_cancel, this, _1),
      std::bind(&Feeder::handle_accepted, this, _1)
    );
  }

private: // action server callbacks

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Message::Goal> goal)
  {
    (void) uuid;
    RCLCPP_INFO(this->get_logger(), "goal: %s", goal->name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "cancel: %s", goal->name.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&Feeder::execute, this, _1), goal_handle }.detach();
  }

private: // private member functions

  void execute(const std::shared_ptr<GoalHandleMessage> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Message::Feedback>();
    rclcpp::Rate loop_rate(goal->period_second);

    RCLCPP_INFO(this->get_logger(), "execute: %s", goal->name.c_str());

    for (uint32_t i = 0; i < goal->feed_count && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Message::Result>();
        result->sended_count = i;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "canceled: %s %u", goal->name.c_str(), i);
        return;
      }

      feedback->feed = goal->name + std::to_string(i);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "feed: %s", goal->name.c_str());

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      auto result = std::make_shared<Message::Result>();
      result->sended_count = goal->feed_count;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "succeed: %s", goal->name.c_str());
    }
  }

private:
  rclcpp_action::Server<Message>::SharedPtr action_server_;

};  // class Feeder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<Feeder>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
