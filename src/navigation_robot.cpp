#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class NavigationRobot : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigationRobot()
  : Node("navigation_robot")
  {
    // Khởi tạo Action Client để gửi lệnh đến Nav2
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    // Subscriber lắng nghe mục tiêu từ nút '2D Goal Pose' trên RViz
    // Lưu ý: Topic mặc định của nút này là /goal_pose
    this->goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&NavigationRobot::on_goal_received, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started. Ready! Click '2D Goal Pose' in RViz to move the robot.");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Hàm xử lý khi nhận được mục tiêu từ RViz
  void on_goal_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received new goal from RViz: x=%.2f, y=%.2f", 
                msg->pose.position.x, msg->pose.position.y);
    
    if (!this->client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available!");
      return;
    }

    // Gửi mục tiêu này đến Nav2 Action Server
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *msg; // Lấy nguyên pose từ RViz

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted! Robot is starting to move.");
      }
    };

    send_goal_options.feedback_callback = [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      static int count = 0;
      if (count++ % 30 == 0) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);
      }
    };

    send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal Reached successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal.");
      }
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationRobot>());
  rclcpp::shutdown();
  return 0;
}