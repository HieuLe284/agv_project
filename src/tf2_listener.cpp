#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "library/common/math_utils.h"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

/**
 * Tf2Listener Node
 * ----------------
 * Minh họa: TF2 trong ROS2 C++
 *
 * Mỗi 1 giây, node sẽ lookup transform từ frame "odom" → "base_link"
 * để lấy vị trí robot trong hệ tọa độ toàn cục (odom frame).
 *
 * TF Tree:  odom → base_link → chassis → [các bánh xe & head]
 *           (do diff_drive plugin + robot_state_publisher publish)
 */
class Tf2Listener : public rclcpp::Node
{
public:
  Tf2Listener()
  : Node("tf2_listener")
  {
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      1s, std::bind(&Tf2Listener::lookupTransform, this));

    RCLCPP_INFO(this->get_logger(), "Tf2Listener node started!");
    RCLCPP_INFO(this->get_logger(), "Listening to TF: odom -> base_link");
  }

private:
  void lookupTransform()
  {
    try {

      geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);

      // Vị trí (translation)
      double tx = t.transform.translation.x;
      double ty = t.transform.translation.y;
      double tz = t.transform.translation.z;

      // Góc quay (quaternion)
      double qx = t.transform.rotation.x;
      double qy = t.transform.rotation.y;
      double qz = t.transform.rotation.z;
      double qw = t.transform.rotation.w;

      // Tính góc yaw từ quaternion
      double yaw = quatToYaw(qx, qy, qz, qw);

      RCLCPP_INFO(this->get_logger(),
        "[TF2] odom→base_link: pos=(%.3f, %.3f, %.3f) | yaw=%.2f rad",
        tx, ty, tz, yaw);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "Transform not available: %s — waiting...", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
  rclcpp::TimerBase::SharedPtr                  timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tf2Listener>());
  rclcpp::shutdown();
  return 0;
}
