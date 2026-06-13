#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "agv_robot/action/move_cmd.hpp"

class MoveActionClient : public rclcpp::Node
{
public:
  using MoveCmd = agv_robot::action::MoveCmd;
  using GoalHandleMoveCmd = rclcpp_action::ClientGoalHandle<MoveCmd>;

  explicit MoveActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MoveCmd>(
      this,
      "move_robot");
  }

  void send_goal(const std::string& command, float value)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = MoveCmd::Goal();
    goal_msg.command = command;
    goal_msg.value = value;

    RCLCPP_INFO(this->get_logger(), "Sending goal: command='%s', value=%.2f", command.c_str(), value);

    auto send_goal_options = rclcpp_action::Client<MoveCmd>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MoveActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MoveActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MoveActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveCmd>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleMoveCmd::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleMoveCmd::SharedPtr,
    const std::shared_ptr<const MoveCmd::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback - Progress: %.2f", feedback->progress);
  }

  void result_callback(const GoalHandleMoveCmd::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: action_robot <command> <value>");
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Commands: up, down, circle");
    return 1;
  }

  std::string command = argv[1];
  float value = std::atof(argv[2]);

  auto action_client = std::make_shared<MoveActionClient>();
  action_client->send_goal(command, value);

  rclcpp::spin(action_client);

  return 0;
}
