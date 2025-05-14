#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "data_assoc_trainee.hpp"

#include <vector>
#include <numeric>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "dv_msgs/msg/indexed_track.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

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

        per_subscription_ = this->create_subscription<dv_msgs::msg::IndexedTrack>(
        "/perception/cones", 10, std::bind(&Localizer::topic_callback4, this, _1));

      //timer_ = this->create_wall_timer( 
      //1ms, std::bind(&Localizer::timer_callback, this));

    }


      Eigen::Matrix3d Q_tl;


      Eigen::Matrix3d sig_tl;

      Eigen::Matrix2d R_t;


    
    float dt = 0.001;


    float x_t, y_t, phi_t, real_x, real_y;
    float vx, vy, w, vrr, vrl;


    std::vector<double> mu_t;

    std::vector<dv_msgs::msg::IndexedCone> conesFromPerception;

    
    Eigen::Matrix3d G_t;
    

    Eigen::Matrix3d V_tl;

    Eigen::Matrix3d Q_t;

    Eigen::Matrix3d sig_t;

    std::vector<int> associated_indices;

    double x_k, y_k, color_k, x_c, y_c;
    double q, q_hat, q_sqrt, q_hat_sqrt;
    std::vector<double> z_t, z_hat, z_diff;
    Eigen::Matrix3d K_t;
    Eigen::Matrix3d H_t;


    std::vector<Eigen::Vector3d> known_landmarks = {
    // Blue cones (color code = 0)
    Eigen::Vector3d(-3.7219, -0.7307, 0),
    Eigen::Vector3d(0.2447, 2.1900, 0),
    Eigen::Vector3d(6.1094, 2.7757, 0),
    Eigen::Vector3d(22.6296, 5.5379, 0),
    Eigen::Vector3d(25.4920, 6.7404, 0),
    Eigen::Vector3d(28.1389, 6.2025, 0),
    Eigen::Vector3d(32.5034, 3.3361, 0),
    Eigen::Vector3d(33.8922, 1.4012, 0),
    Eigen::Vector3d(30.5221, 5.0812, 0),
    Eigen::Vector3d(34.4761, -1.5043, 0),
    Eigen::Vector3d(33.9992, -5.0234, 0),
    Eigen::Vector3d(32.9925, -8.0595, 0),
    Eigen::Vector3d(32.0983, -10.3000, 0),
    Eigen::Vector3d(30.1824, -13.5399, 0),
    Eigen::Vector3d(24.1140, -17.0441, 0),
    Eigen::Vector3d(8.9345, 3.0637, 0),
    Eigen::Vector3d(27.2831, -15.5644, 0),
    Eigen::Vector3d(21.2536, -18.8389, 0),
    Eigen::Vector3d(18.0618, -20.4551, 0),
    Eigen::Vector3d(14.4209, -22.2634, 0),
    Eigen::Vector3d(10.5025, -24.3305, 0),
    Eigen::Vector3d(7.2514, -26.4217, 0),
    Eigen::Vector3d(3.6516, -27.4551, 0),
    Eigen::Vector3d(0.7886, -26.9459, 0),
    Eigen::Vector3d(-1.4625, -25.2249, 0),
    Eigen::Vector3d(-3.2427, -23.5760, 0),
    Eigen::Vector3d(12.8688, 3.0125, 0),
    Eigen::Vector3d(-5.1431, -21.3860, 0),
    Eigen::Vector3d(-5.6174, -17.8608, 0),
    Eigen::Vector3d(-5.9382, -15.4551, 0),
    Eigen::Vector3d(-5.5558, -12.8602, 0),
    Eigen::Vector3d(-5.1206, -10.3000, 0),
    Eigen::Vector3d(-4.7841, -7.2575, 0),
    Eigen::Vector3d(-4.8432, -4.0291, 0),
    Eigen::Vector3d(-2.1864, 1.4137, 0),
    Eigen::Vector3d(16.5042, 3.4245, 0),
    Eigen::Vector3d(20.1368, 4.4270, 0),

    // Yellow cones (color code = 1)
    Eigen::Vector3d(0.7816, -2.6920, 1),
    Eigen::Vector3d(20.1479, -0.3734, 1),
    Eigen::Vector3d(23.4312, 0.4990, 1),
    Eigen::Vector3d(25.9655, 1.5014, 1),
    Eigen::Vector3d(27.9652, 0.9833, 1),
    Eigen::Vector3d(29.6054, -1.1797, 1),
    Eigen::Vector3d(6.0900, -1.8555, 1),
    Eigen::Vector3d(29.7063, -4.1923, 1),
    Eigen::Vector3d(28.8760, -6.8209, 1),
    Eigen::Vector3d(28.1060, -8.7973, 1),
    Eigen::Vector3d(26.6765, -10.3000, 1),
    Eigen::Vector3d(22.4195, -13.1474, 1),
    Eigen::Vector3d(25.0732, -11.7312, 1),
    Eigen::Vector3d(20.2628, -14.2408, 1),
    Eigen::Vector3d(17.6516, -15.4551, 1),
    Eigen::Vector3d(14.7877, -16.9672, 1),
    Eigen::Vector3d(11.0203, -18.8633, 1),
    Eigen::Vector3d(8.9831, -1.7968, 1),
    Eigen::Vector3d(7.8188, -20.8555, 1),
    Eigen::Vector3d(5.4296, -22.4125, 1),
    Eigen::Vector3d(3.2361, -23.0081, 1),
    Eigen::Vector3d(0.9662, -21.9718, 1),
    Eigen::Vector3d(-0.9298, -19.2829, 1),
    Eigen::Vector3d(-1.1575, -16.0733, 1),
    Eigen::Vector3d(-1.0043, -12.9300, 1),
    Eigen::Vector3d(-0.6087, -10.3000, 1),
    Eigen::Vector3d(-0.3478, -7.2429, 1),
    Eigen::Vector3d(12.9402, -1.9241, 1),
    Eigen::Vector3d(-0.3455, -4.5119, 1),
    Eigen::Vector3d(16.5151, -1.6032, 1),

    // Big Orange (color code = 2)
    Eigen::Vector3d(3.3869, 2.7000, 2),
    Eigen::Vector3d(3.0007, 2.6890, 2),
    Eigen::Vector3d(3.3785, -1.9068, 2),
    Eigen::Vector3d(3.0133, -1.9065, 2),
};



  private:
    void topic_callback1(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      w = msg->angular_velocity.z;
      RCLCPP_INFO(this->get_logger(), "IMU angular velocity z: %.2f", w); 
    }
    
    void topic_callback2(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "topic_callback2 triggered");
      vrl = msg -> speeds.lb_speed;
      vrr = msg -> speeds.rb_speed;
      vx = (3.14 * 0.25) * (vrr + vrl)/60.0;
      vy = 0.0;
    }

    void topic_callback3(const eufs_msgs::msg::CarState::SharedPtr msg)
    {
      real_x = msg -> pose.pose.position.x;
      real_y = msg -> pose.pose.position.y;
    }
    

    void topic_callback4(const dv_msgs::msg::IndexedTrack & msg)
    {

      x_t = x_t + 0.001 * (cos(phi_t)*vx - sin(phi_t)*vy);
      y_t = y_t + 0.001 * (cos(phi_t)*vy + sin(phi_t)*vx);
      phi_t = phi_t + w * 0.001;

      mu_t = {x_t, y_t, phi_t};
      auto message = std_msgs::msg::Float32MultiArray();

        R_t << 0.001, 0.0,
        0.0, 0.001;

        Q_tl << 0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001;

        sig_tl << 0.001, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3;

      
        G_t << 1, 0, (vx*sin(phi_t)-vy*cos(phi_t))*dt,
           0, 1, (vx*cos(phi_t)+vy*sin(phi_t))*dt,
           0, 0, dt;

        V_tl << dt*cos(phi_t), -dt*sin(phi_t) , 0.0,
            dt*sin(phi_t), dt*cos(phi_t), 0.0,
            0.0, 0.0, 1;

        Q_t = V_tl * Q_tl * V_tl.transpose();

        sig_t = G_t * sig_tl * G_t.transpose() + Q_tl;   
        
        


      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.cones[0].color_code.c_str());
      conesFromPerception = msg.track;
      
      std::vector<int> associated_indices = performDataAssociation(mu_t, conesFromPerception, R_t);
      for (size_t i = 0; i < associated_indices.size(); ++i) {
          //RCLCPP_INFO(this->get_logger(), "Associated index: %d", associated_indices[i]);

        x_k = known_landmarks[associated_indices[i]][0];
        y_k = known_landmarks[associated_indices[i]][1];
        color_k = known_landmarks[associated_indices[i]][2];

        x_c = conesFromPerception[i].location.x;
        y_c = conesFromPerception[i].location.y;

        q = (x_k - x_t)*(x_k - x_t) + (y_k - y_t)*(y_k - y_t);
        q_sqrt = sqrt(q);
        
        std::vector<double> z_t = {q_sqrt, atan2(y_k - y_t, x_k - x_t)};
        
        q_hat = x_c*x_c + y_c*y_c;
        q_hat_sqrt = sqrt(q_hat);
        z_hat = {q_hat_sqrt, atan2(y_c, x_c)};

        Eigen::Vector2d z_diff;
            z_diff << z_t[0] - z_hat[0], (z_t[1] - z_hat[1]);

        

        Eigen::Matrix<double, 2, 3> H_t;
        H_t << -(x_k - x_t)/q_sqrt, -(y_k - y_t)/q_sqrt, 0,
                (y_k - y_t)/q, -(x_k - x_t)/q, -1;

        Eigen::Matrix<double, 3, 2>  K_t = sig_t * H_t.transpose() * (H_t * sig_t * H_t.transpose() + R_t).inverse();

        Eigen::Vector3d mu_eig(mu_t[0], mu_t[1], mu_t[2]);


        Eigen::Vector3d updated_mu = mu_eig + K_t * z_diff;

        mu_t[0] = updated_mu(0);
        mu_t[1] = updated_mu(1);
        mu_t[2] = updated_mu(2);

        sig_t = (Eigen::Matrix3d::Identity() - K_t * H_t) * sig_t;

      }

      message.data.push_back(mu_t[0]);
      message.data.push_back(mu_t[1]);
      message.data.push_back(mu_t[2]);
      message.data.push_back(real_x);
      message.data.push_back(real_y);

      RCLCPP_INFO(this->get_logger(), "Publishing pose: [%.2f, %.2f, %.2f]", x_t, y_t, phi_t);
      publisher_->publish(message);

    }

    /*void timer_callback()
    {

      x_t = x_t + 0.001 * (cos(phi_t)*vx - sin(phi_t)*vy);
      y_t = y_t + 0.001 * (cos(phi_t)*vy + sin(phi_t)*vx);
      phi_t = phi_t + w * 0.001;

      mu_t = {x_t, y_t, phi_t};
      auto message = std_msgs::msg::Float32MultiArray();
      message.data.push_back(mu_t[0]);
      message.data.push_back(mu_t[1]);
      message.data.push_back(mu_t[2]);
      message.data.push_back(real_x);
      message.data.push_back(real_y);

      RCLCPP_INFO(this->get_logger(), "Publishing pose: [%.2f, %.2f, %.2f]", x_t, y_t, phi_t);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      R_t << 0.001, 0.0,
        0.0, 0.001;

      Q_tl << 0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001;

      sig_tl << 0.001, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3;

      
        G_t << 1, 0, (vx*sin(phi_t)-vy*cos(phi_t))*dt,
           0, 1, (vx*cos(phi_t)+vy*sin(phi_t))*dt,
           0, 0, dt;

        V_tl << dt*cos(phi_t), -dt*sin(phi_t) , 0.0,
            dt*sin(phi_t), dt*cos(phi_t), 0.0,
            0.0, 0.0, 1;

        Q_t = V_tl * Q_tl * V_tl.transpose();

        sig_t = G_t * sig_tl * G_t.transpose() + Q_tl;        
    } */

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr gr_subscription_;
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr re_subscription_;
    rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr per_subscription_;
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