#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cmath>

using std::placeholders::_1;

class PublishingSubscriber : public rclcpp::Node
{
  public:
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic_fk", 10);
      // timer_ = this->create_wall_timer(
      // 1000ms, std::bind(&MinimalSubscriber::topic_callback, this));
    }
    
    // MinimalPublisher()
    // : Node("minimal_publisher"), count_(0)
    // {
    //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic_fk", 10);
    //   timer_ = this->create_wall_timer(
    //   1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    // }


  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {
      float m[4][4];
      
      float q1 = msg.position[0]; 
      float q2 = msg.position[1];
      float q3 = msg.position[2];

      float L1 = 1.0;
      float L2 = 1.0;
      float L3 = 1.0;
      
      m[0][0] = -cos(q1+q2);
      m[0][1] = -sin(q1+q2);
      m[0][2] = 0.0;
      m[0][3] = cos(q1+q2) + 0.9*cos(q1);
      m[1][0] = -sin(q1+q2);
      m[1][1] = cos(q1+q2);
      m[1][2] = 0.0;
      m[1][3] = sin(q1+q2) + 0.9*sin(q1);
      m[2][0] = 0.0;
      m[2][1] = 0.0;
      m[2][2] = -1.0;
      m[2][3] = 1 + q3;
      m[3][0] = 0.0;
      m[3][1] = 0.0;
      m[3][2] = 0.0;
      m[3][3] = 1.0;
      
      std::string mat;
      for (int i=0; i<4;i++)
      {
        mat+="[";
      for (int j=0; j<4;j++)
      {
      mat += std::to_string(m[i][j]) + " \t";
      }
      mat+=" ] \n";
      }
      //RCLCPP_INFO(this->get_logger(), "%s", s.c_str());

      // Create a new message of type String
      auto result = std_msgs::msg::String();
      result.data = mat;
      publisher_->publish(result);

    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
