#include "rclcpp/rclcpp.hpp"
#include "proj_fk/srv/calc_ik.hpp"

#include <memory>

void IK(const std::shared_ptr<proj_fk::srv::CalcIK::Request> request,
          std::shared_ptr<proj_fk::srv::CalcIK::Response>      response)
{
  float x = request->x;
  float y = request->y;
  float z = request->z;
  float Lb = 1.0, L1=0.9, L2=1.0 ,L3=0.95;
  float q1,q2,q3,q1_2,q2_2;
  float c2 = (x*x + y*y - L1*L1 - L2*L2)/(2*L1*L2);
  float s2=0;
  if(abs(c2)<=1){
    s2 = sqrt(1-c2*c2);
  }
  q2 = atan2(s2,c2);
  q3 = z-Lb;
  q1 = atan2(y,x) - atan2(L2*s2,(L1+L2*c2));

  q2_2 = -q2;
  q1_2 = atan2(y,x) - atan2(L2*(-1*s2),(L1+L2*c2));

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f,%f,%f",q1,q2,q3);
  response->q1=q1;
  response->q2=q2;
  response->q3=q3;
  response->q2_2=q2_2;
  response->q1_2=q1_2;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("IK_server");

  rclcpp::Service<proj_fk::srv::CalcIK>::SharedPtr service =
    node->create_service<proj_fk::srv::CalcIK>("calc_IK", &IK);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate IK.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}


// float x_c = msg.position.x;  //input values (end-effector position) from terminal
//       float y_c = msg.position.y;
//       float z_c = msg.position.z;
      
//       float L1 = 1.0;
//       float L2 = 1.0;
//       float L3 = 1.0;
      
//       float D = (pow(x_c,2) + pow(y_c,2) + pow(z_c - L1,2) - pow(L2,2) - pow(L3,2))/(2*L2*L3);
      
//       //calculating joint angles
//       float q3_1 = -1*atan2(sqrt(1-pow(D,2)),D);//one solution of q3
//       float q3_2 = -1*atan2(-sqrt(1-pow(D,2)),D); //another solution of q3
//       float q2_1 = -1*atan2(z_c-L1,sqrt(pow(x_c,2) + pow(y_c,2))) - atan2(L3*sin(q3_1),L2 + L3*cos(q3_1));//one solution of q2
//       float q2_2 = -1*atan2(z_c-L1,sqrt(pow(x_c,2) + pow(y_c,2))) - atan2(L3*sin(q3_2),L2 + L3*cos(q3_2));//another solution of q2
//       float q1 = atan2(y_c,x_c);
      
      
//       RCLCPP_INFO(this->get_logger(), "First set of joint angles: '%f','%f','%f'", q1,q2_1,q3_1); //printing joint angles
//       RCLCPP_INFO(this->get_logger(), "Second set of joint angles: '%f','%f','%f'", q1,q2_2,q3_2); //printing joint angles

//     }
    
//     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_1;
//     rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_2;