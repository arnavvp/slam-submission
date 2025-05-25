#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "dv_msgs/msg/indexed_track.hpp"
#include "eufs_msgs/msg/car_state.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<dv_msgs::msg::IndexedTrack>(
      "/perception/cones", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      ground_subscription_ = this->create_subscription<eufs_msgs::msg::CarState>(
      "/ground_truth/state", 10, std::bind(&MinimalSubscriber::ground_callback, this, _1));

      state_vector = {0.0, 0.0, 0.0};

      
        
    }

  private:

    std::vector<double> state_vector;
    std::vector<double> associated_indices;

    void topic_callback(const dv_msgs::msg::IndexedTrack & msg)
    {
        associated_indices.clear();

        for (const auto & cone : msg.track) {
            bool is_associated = false;
            if (state_vector.size() == 3) {
                state_vector.push_back(cone.location.x);
                state_vector.push_back(cone.location.y);
                
            }
            else{
                for(size_t i = 3; i < state_vector.size(); i+=2){
                    double yaw = state_vector[2];
                    double cone_perc_x = state_vector[0] + std::cos(yaw) * cone.location.x - std::sin(yaw) * cone.location.y;
                    double cone_perc_y = state_vector[1] + std::sin(yaw) * cone.location.x + std::cos(yaw) * cone.location.y;

                    double dist = sqrt((cone_perc_x-state_vector[i])*(cone_perc_x-state_vector[i])+(cone_perc_y-state_vector[i+1])*(cone_perc_y-state_vector[i+1]));
                    if(dist < 0.5){
                        is_associated = true;
                        associated_indices.push_back(i);
                        break;
                    }
                }
                if(!is_associated){
                    state_vector.push_back(cone.location.x);
                    state_vector.push_back(cone.location.y);
                }

            }
        }

        std::stringstream ss;
    ss << "Associated indices: ";
    for (const auto & index : associated_indices) {
        ss << index << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void ground_callback(const eufs_msgs::msg::CarState & msg)
    {
        state_vector[0] = msg.pose.pose.position.x;
        state_vector[1] = msg.pose.pose.position.y;
        const auto & q = msg.pose.pose.orientation;
        state_vector[2] = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr ground_subscription_;
    rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
