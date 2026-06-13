#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>

#define KEYCODE_UP    0x41  // ↑  tiến
#define KEYCODE_DOWN  0x42  // ↓  lùi
#define KEYCODE_RIGHT 0x43  // →  quay phải
#define KEYCODE_LEFT  0x44  // ←  quay trái
#define KEYCODE_STOP  0x73  // s  dừng
#define KEYCODE_Q     0x71  // q  dừng & thoát


int  kfd = 0;
struct termios cooked, raw;

void quit(int /*sig*/)
{
  tcsetattr(kfd, TCSANOW, &cooked); 
  rclcpp::shutdown();
  exit(0);
}

class RobotController : public rclcpp::Node
{
public:
  RobotController()
  : Node("robot_controller"),
    linear_scale_(1.0),
    angular_scale_(1.0)
  {
    this->declare_parameter("scale_linear",  linear_scale_);
    this->declare_parameter("scale_angular", angular_scale_);
    linear_scale_  = this->get_parameter("scale_linear").as_double();
    angular_scale_ = this->get_parameter("scale_angular").as_double();

    // Publisher → /cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "RobotController started.... Sử dụng các phím sau để di chuyển.");
    RCLCPP_INFO(this->get_logger(), "  ↑ / ↓  : tiến / lùi  (scale: %.1f)", linear_scale_);
    RCLCPP_INFO(this->get_logger(), "  ← / →  : trái / phải (scale: %.1f)", angular_scale_);
    RCLCPP_INFO(this->get_logger(), "  s       : dừng");
    RCLCPP_INFO(this->get_logger(), "  q       : dừng & thoát");
  }

  void keyLoop()
  {
    char c;

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("\n--------------------------------------------------");
    puts("  Reading from keyboard — Mobile Robot Controller");
    puts("--------------------------------------------------");
    puts("  Arrow keys : di chuyển");
    puts("  s          : dừng");
    puts("  q          : dừng & thoát\n");

    bool dirty = false;
    double current_linear  = 0.0;
    double current_angular = 0.0;

    for (;;) {
      if (read(kfd, &c, 1) < 0) {
        perror("read():");
        exit(-1);
      }

      if (c == 0x1B) {
        char seq[2];
        if (read(kfd, &seq[0], 1) < 0) continue;
        if (read(kfd, &seq[1], 1) < 0) continue;

        if (seq[0] == '[') c = seq[1];
      }

      switch (c) {
        case KEYCODE_UP:
          std::cout << "UP" << std::endl;
          current_linear += 0.1;  
          if (current_linear > 2.0) current_linear = 2.0;
          dirty = true;
          break;
        case KEYCODE_DOWN:
          std::cout << "DOWN" << std::endl;
          current_linear -= 0.1;  
          if (current_linear < -2.0) current_linear = -2.0;
          dirty = true;
          break;
        case KEYCODE_LEFT:
          std::cout << "LEFT" << std::endl;
          current_angular -= 0.1; 
          if (current_angular < -2.0) current_angular = -2.0;
          dirty = true;
          break;
        case KEYCODE_RIGHT:
          std::cout << "RIGHT" << std::endl;
          current_angular += 0.1; 
          if (current_angular > +2.0) current_angular = +2.0;
          dirty = true;
          break;
        case KEYCODE_STOP:
          std::cout << "STOP" << std::endl;
          current_linear  = 0.0;
          current_angular = 0.0;
          dirty = true;
          break;
        case KEYCODE_Q:
          RCLCPP_INFO(this->get_logger(), "Quit — stopping robot.");
          publishZero();
          quit(0);
          break;
        default:
          break;
      }

      if (dirty) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x  = linear_scale_  * current_linear;
        msg.angular.z = angular_scale_ * current_angular;
        pub_->publish(msg);
        dirty = false;
        
        printf("\rCurrent velocity: linear = %.2f, angular = %.2f   ", 
                msg.linear.x, msg.angular.z);
        fflush(stdout);
      }
    }
  }

private:
  void publishZero()
  {
    pub_->publish(geometry_msgs::msg::Twist()); 
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  double linear_scale_;
  double angular_scale_;
};

// ============================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, quit);

  auto node = std::make_shared<RobotController>();
  node->keyLoop();  
  rclcpp::shutdown();
  return 0;
}