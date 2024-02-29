#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <csignal>

// 定义键盘输入宏定义
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double linear_x, linear_y, angular_;
  double l_scale_, a_scale_;
  ros::Publisher twist_pub_;
};

TeleopTurtle::TeleopTurtle() : linear_x(0), linear_y(0), angular_(0), l_scale_(2.0), a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT, quit);

  teleop_turtle.keyLoop();

  return (0);
}

void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' keys to control the turtle.")
  puts("'Q/E' to increase/decrease speed.");

  for (;;)
  {
    // read the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_x = linear_y = angular_ = 0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch (c)
    {
      case KEYCODE_W:
        ROS_DEBUG("UP");
        linear_x = 0.3;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("DOWN");
        linear_x = -0.3;
        dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_Z:
        ROS_DEBUG("MOVE_RIGHT");
        linear_y = 0.5;
        dirty = true;
        break;
      case KEYCODE_X:
        ROS_DEBUG("MOVE_LEFT");
        linear_y = -0.5;
        dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("QUICK");
        l_scale_ += 0.2;
        a_scale_ += 0.2;
        dirty = true;
        break;
      case KEYCODE_E:
        ROS_DEBUG("SLOW");
        l_scale_ -= 0.2;
        a_scale_ -= 0.2;
        dirty = true;
        break;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * angular_;
    twist.linear.x = l_scale_ * linear_x;
    twist.linear.y = l_scale_ * linear_y;
    if (dirty == true)
    {
      twist_pub_.publish(twist);
      dirty = false;
    }
  }

  return;
}
