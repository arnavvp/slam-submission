#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;




class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher() : Node("marker_publisher")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    re_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("re_visualization_marker", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MarkerPublisher::publish_marker, this));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "current_pose", 10, std::bind(&MarkerPublisher::topic_callback, this, _1));
    
    } 
    float x_t, y_t, phi_t, real_x, real_y;

private:

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.angular_velocity.z.c_str());

     if (msg->data.size() < 5) {
    RCLCPP_WARN(this->get_logger(), "Received msg with insufficient data size: %zu", msg->data.size());
    return;
  }

    x_t = msg->data[0];
    y_t = msg->data[1];
    real_x = msg->data[3];
    real_y = msg->data[4];


    visualization_msgs::msg::Marker re_marker;
    re_marker.header.frame_id = "map";  
    re_marker.header.stamp = this->get_clock()->now();

    re_marker.ns = "basic_shapes";
    re_marker.id = 0;
    re_marker.type = visualization_msgs::msg::Marker::ARROW;
    re_marker.action = visualization_msgs::msg::Marker::ADD;

    re_marker.pose.position.x = 0;
    re_marker.pose.position.y = 0;
    re_marker.pose.position.z = 0;
    re_marker.pose.orientation.x = 0.0;
    re_marker.pose.orientation.y = 0.0;
    re_marker.pose.orientation.z = 0.0;
    re_marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point re_start, re_end;
    re_start.x = 0.0;
    re_start.y = 0.0;
    re_start.z = 0.0;

    re_end.x = real_x;  
    re_end.y = real_y;
    re_end.z = 0.0;

    re_marker.points.push_back(re_start);
    re_marker.points.push_back(re_end);

    re_marker.scale.x = 0.1;
    re_marker.scale.y = 0.2;
    re_marker.scale.z = 0.2;

    re_marker.color.r = 0.0f;
    re_marker.color.g = 0.0f;
    re_marker.color.b = 1.0f;
    re_marker.color.a = 1.0;

    re_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    re_marker_pub_->publish(re_marker);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // no leading slash
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    end.x = x_t;  // Change this as needed
    end.y = y_t;
    end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    marker_pub_->publish(marker);
    }

  void publish_marker()
  {
    

    
  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr re_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}