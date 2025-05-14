#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"


using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Localizer : public rclcpp::Node
{
  public:
    Localizer()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("current_pose", 10);
      

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&Localizer::topic_callback1, this, _1));

      gr_subscription_ = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
        "/ros_can/wheel_speeds", 10, std::bind(&Localizer::topic_callback2, this, _1));

      re_subscription_ = this->create_subscription<eufs_msgs::msg::CarState>(
        "/ground_truth/state", 10, std::bind(&Localizer::topic_callback3, this, _1));

      timer_ = this->create_wall_timer( 
      1ms, std::bind(&Localizer::timer_callback, this));
    }
    float x_t, y_t, phi_t, real_x, real_y;
    float vx, vy, w, vrr, vrl;


  private:
    void topic_callback1(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.angular_velocity.z.c_str());
      w = msg->angular_velocity.z;
      RCLCPP_INFO(this->get_logger(), "IMU angular velocity z: %.2f", w);
      
    }
    
    void topic_callback2(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
    {
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.angular_velocity.z.c_str());
      RCLCPP_INFO(this->get_logger(), "topic_callback2 triggered");
      vrl = msg -> speeds.lb_speed;
      vrr = msg -> speeds.rb_speed;
      vx = (3.14 * 0.25) * (vrr + vrl)/60.0;
      vy = 0.0;
      //RCLCPP_INFO(this->get_logger(), "velocities: [%.2f, %.2f, %.2f, %.2f]", vrr, vrl, vx, vy);
      //real_x = msg -> pose.pose.position.x;
      //real_y = msg -> pose.pose.position.y;
    }

    void topic_callback3(const eufs_msgs::msg::CarState::SharedPtr msg)
    {
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.angular_velocity.z.c_str());
      real_x = msg -> pose.pose.position.x;
      real_y = msg -> pose.pose.position.y;

    }
    
    
    /*void calculator(){
      
      
      x_t = x_t + 0.5 * (cos(phi_t)*vx - sin(phi_t)*vy);
      y_t = y_t + 0.5 * (cos(phi_t)*vy + sin(phi_t)*vx);
      phi_t = phi_t + w * 0.5;
    }*/


    void timer_callback()
    {

      x_t = x_t + 0.001 * (cos(phi_t)*vx - sin(phi_t)*vy);
      y_t = y_t + 0.001 * (cos(phi_t)*vy + sin(phi_t)*vx);
      phi_t = phi_t + w * 0.001;

      auto message = std_msgs::msg::Float32MultiArray();
      message.data.push_back(x_t);
      message.data.push_back(y_t);
      message.data.push_back(real_x);
      message.data.push_back(real_y);
      message.data.push_back(phi_t);

      //RCLCPP_INFO(this->get_logger(), "velocities: [%.2f, %.2f, %.2f, %.2f]", vrr, vrl, vx, vy);

      RCLCPP_INFO(this->get_logger(), "Publishing pose: [%.2f, %.2f, %.2f]", x_t, y_t, phi_t);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr gr_subscription_;
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr re_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
    
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localizer>());
  rclcpp::shutdown();
  return 0;
}