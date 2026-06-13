#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "agv_robot/action/move_cmd.hpp"
#include "library/common/math_utils.h"

class MoveActionServer : public rclcpp::Node
{
public:
  using MoveCmd = agv_robot::action::MoveCmd;
  using GoalHandleMoveCmd = rclcpp_action::ServerGoalHandle<MoveCmd>;

  MoveActionServer()
  : Node("move_action_server")
  {
    using namespace std::placeholders;

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MoveActionServer::odomCallback, this, _1));

    action_server_ = rclcpp_action::create_server<MoveCmd>(
      this,
      "move_robot",
      std::bind(&MoveActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveActionServer::handle_cancel, this, _1),
      std::bind(&MoveActionServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "MoveActionServer is ready.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<MoveCmd>::SharedPtr action_server_;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = quatToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveCmd::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: command='%s', value=%.2f",
      goal->command.c_str(), goal->value);
    (void)uuid;
    
    if (goal->command != "up" && goal->command != "down" && goal->command != "circle") {
      RCLCPP_ERROR(this->get_logger(), "Invalid command. Rejecting goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveCmd> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveCmd> goal_handle)
  {
    std::thread(&MoveActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveCmd> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    rclcpp::Rate loop_rate(10); // 10Hz

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveCmd::Feedback>();
    auto result = std::make_shared<MoveCmd::Result>();

    auto cmd = geometry_msgs::msg::Twist();
    
    double start_x = current_x_;
    double start_y = current_y_;
    double last_yaw = current_yaw_;
    
    double accumulated_progress = 0.0;
    
    if (goal->command == "up") {
      cmd.linear.x = 0.5;
    } else if (goal->command == "down") {
      cmd.linear.x = -0.5;
    } else if (goal->command == "circle") {
      cmd.linear.x = 0.5;
      cmd.angular.z = 0.5;
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        stop_robot();
        return;
      }

      if (goal->command == "up" || goal->command == "down") {
        accumulated_progress = std::sqrt(std::pow(current_x_ - start_x, 2) + std::pow(current_y_ - start_y, 2));
      } else if (goal->command == "circle") {
        double diff = current_yaw_ - last_yaw;
        if (diff > M_PI) diff -= 2 * M_PI;
        if (diff < -M_PI) diff += 2 * M_PI;
        accumulated_progress += std::abs(diff);
        last_yaw = current_yaw_;
      }

      cmd_pub_->publish(cmd);

      feedback->progress = accumulated_progress;
      goal_handle->publish_feedback(feedback);

      if (accumulated_progress >= goal->value) {
        break;
      }

      loop_rate.sleep();
    }

    stop_robot();

    if (rclcpp::ok()) {
      result->success = true;
      result->message = "Hoan thanh viec di chuyen!";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }
  }
  
  void stop_robot() {
    auto stop_cmd = geometry_msgs::msg::Twist();
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_pub_->publish(stop_cmd);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
