#include "rclcpp/rclcpp.hpp"
#include "proj_fk/srv/jvel.hpp"
#include "proj_fk/srv/eevel.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>



using namespace std;

using std::placeholders::_1;

class PublishingSubscriber : public rclcpp::Node
{
  public:
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);

      service_ee = this->create_service<proj_fk::srv::Eevel>("Joint2EE_vel", std::bind(&PublishingSubscriber::ee_vel, this, _1, std::placeholders::_2));
      
      service_j = this->create_service<proj_fk::srv::Jvel>("EE2Joint_vel", std::bind(&PublishingSubscriber::j_vel, this, _1, std::placeholders::_2));
       
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to receive joint values.");


   }
    
   

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
    {
         //RCLCPP_INFO(this->get_logger(), "haga 1");
      q1 = msg.position[0]; 
      q2 = msg.position[1];
      q3 = msg.position[2];

      // v1 = msg.velocity[0]; 
      // v2 = msg.ve[1];
      // v3 = msg.position[2];

      float v1 = msg.velocity[0]; 
      float v2 = msg.velocity[1];
      float v3 = msg.velocity[2];

      float tme = msg.header.stamp.sec*1000 + msg.header.stamp.nanosec/1000000;

      float J_i[3][3];

      if (abs(sin(q2)) < 0.01)
      {
        J_i[0][0] = -0.21;
        J_i[0][1] = 0.21;
        J_i[0][2] = 0;
        J_i[1][0] = 0.21;
        J_i[1][1] = 0.21;
        J_i[1][2] = 0;
        J_i[2][0] = 0;
        J_i[2][1] = 0;
        J_i[2][2] = -1;
      }
      else{
        // inverse jacobian
        J_i[0][0] = (10*cos(q1 + q2))/(9*sin(q2));
        J_i[0][1] = (10*sin(q1 + q2))/(9*sin(q2));
        J_i[0][2] = 0;
        J_i[1][0] = -(10*cos(q1 + q2) + 9*cos(q1))/(9*sin(q2));
        J_i[1][1] = -(10*sin(q1 + q2) + 9*sin(q1))/(9*sin(q2));
        J_i[1][2] = 0;
        J_i[2][0] = 0;
        J_i[2][1] = 0;
        J_i[2][2] = -1;
         }



  //matrix multiplication
  
  j1v = J_i[0][0] * x_v + J_i[0][1] * y_v + J_i[0][2] * z_v;
  j2v = J_i[1][0] * x_v + J_i[1][1] * y_v + J_i[1][2] * z_v;
  j3v = J_i[2][0] * x_v + J_i[2][1] * y_v + J_i[2][2] * z_v;

  float J[6][3];
  J[0][0] = -0.9*sin(q1) - sin(q1+q2);
  J[0][1] = -sin(q1+q2);
  J[0][2] = 0;
  J[1][0] = 0.9*cos(q1) + cos(q1+q2);
  J[1][1] = cos(q1+q2);
  J[1][2] = 0;
  J[2][0] = 0;
  J[2][1] = 0;
  J[2][2] = -1;
  J[3][0] = 0;
  J[3][1] = 0;
  J[3][2] = 0;
  J[4][0] = 0;
  J[4][1] = 0;
  J[4][2] = 0;
  J[5][0] = 1;
  J[5][1] = 1;
  J[5][2] = 0;

  // float xn_v = J[0][0] * j1v + J[0][1] * j2v + J[0][2] * j3v;
  // float yn_v = J[1][0] * j1v + J[1][1] * j2v + J[1][2] * j3v;
  // float zn_v = J[2][0] * j1v + J[2][1] * j2v + J[2][2] * j3v;
  
  esum[0] += (j1v-v1)*(tme-old_tme)/1000;
  esum[1] += (j2v-v2)*(tme-old_tme)/1000;
  esum[2] += (j3v-v3)*(tme-old_tme)/1000;

    // PI controller
      float c_v_1 = Kp*(j1v-v1) + Ki*(esum[0]);
      float c_v_2 = Kp*(j2v-v2) + Ki*(esum[1]);
      float c_v_3 = 9.81 + Kp*(j3v-v3) + Ki*(esum[2]);

    // // P controller
    // float c_ef_1 = Kp*(j1v-v1);
    // float c_ef_2 = Kp*(j2v-v2);
    // float c_ef_3 = 9.81 + Kp*(j3v-v3);

     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f,%f,%f",x_v,y_v,z_v);
    //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f,%f,%f",xn_v,yn_v,zn_v);

      auto result = std_msgs::msg::Float64MultiArray();
    //   result.data = {0, 0, 0};
      result.data = {c_v_1, c_v_2, c_v_3};

      publisher_->publish(result);

      // std::ofstream o("/home/mayank/rbe500-ros/save.txt");

     //Jacobian Matrix
  //    float J[6][3];
  // J[0][0] = -0.9*sin(q1) - sin(q1+q2);
  // J[0][1] = -sin(q1+q2);
  // J[0][2] = 0;
  // J[1][0] = 0.9*cos(q1) + cos(q1+q2);
  // J[1][1] = cos(q1+q2);
  // J[1][2] = 0;
  // J[2][0] = 0;
  // J[2][1] = 0;
  // J[2][2] = -1;
  // J[3][0] = 0;
  // J[3][1] = 0;
  // J[3][2] = 0;
  // J[4][0] = 0;
  // J[4][1] = 0;
  // J[4][2] = 0;
  // J[5][0] = 1;
  // J[5][1] = 1;
  // J[5][2] = 0;

//Matrix multiplication
  float x_v_c = J[0][0] * v1 + J[0][1] * v2 + J[0][2] * v3;
  float y_v_c = J[1][0] * v1 + J[1][1] * v2 + J[1][2] * v3;
  float z_v_c = J[2][0] * v1 + J[2][1] * v2 + J[2][2] * v3;
  float x_w = J[3][0] * v1 + J[3][1] * v2 + J[3][2] * v3;
  float y_w = J[4][0] * v1 + J[4][1] * v2 + J[4][2] * v3;
  float z_w = J[5][0] * v1 + J[5][1] * v2 + J[5][2] * v3;



      //Create and open a text file
      std::ofstream MyFile("/home/blacksnow/Desktop/rbe500-ros/src/proj_fk/src/filename1.txt",std::ofstream::app);

      // Write to the file
      MyFile<<x_v<<";"<<x_v_c<<";"<<y_v<<";"<<y_v_c<<";"<<z_v<<";"<<z_v_c<<";"<<tme<<endl;



      // Close the file
      MyFile.close();
      old_tme = tme;

    }

void ee_vel(const std::shared_ptr<proj_fk::srv::Eevel::Request> request,
          std::shared_ptr<proj_fk::srv::Eevel::Response>      response)
{
  float J[6][3];
  memset(J,0,sizeof(J));
  float j1v = request->j1v;
  float j2v = request->j2v;
  float j3v = request->j3v;
  //float j4w = request->j4w;
  //float j5w = request->j4w;
  //float j6w = request->j4w;
  
  
//Jacobian Matrix
  J[0][0] = -0.9*sin(q1) - sin(q1+q2);
  J[0][1] = -sin(q1+q2);
  J[0][2] = 0;
  J[1][0] = 0.9*cos(q1) + cos(q1+q2);
  J[1][1] = cos(q1+q2);
  J[1][2] = 0;
  J[2][0] = 0;
  J[2][1] = 0;
  J[2][2] = -1;
  J[3][0] = 0;
  J[3][1] = 0;
  J[3][2] = 0;
  J[4][0] = 0;
  J[4][1] = 0;
  J[4][2] = 0;
  J[5][0] = 1;
  J[5][1] = 1;
  J[5][2] = 0;

//Matrix multiplication
  float x_v = J[0][0] * j1v + J[0][1] * j2v + J[0][2] * j3v;
  float y_v = J[1][0] * j1v + J[1][1] * j2v + J[1][2] * j3v;
  float z_v = J[2][0] * j1v + J[2][1] * j2v + J[2][2] * j3v;
  float x_w = J[3][0] * j1v + J[3][1] * j2v + J[3][2] * j3v;
  float y_w = J[4][0] * j1v + J[4][1] * j2v + J[4][2] * j3v;
  float z_w = J[5][0] * j1v + J[5][1] * j2v + J[5][2] * j3v;	
   
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f,%f,%f",q1,q2,q3);
  response->x_v=x_v;
  response->y_v=y_v;
  response->z_v=z_v;  
  response->x_w=x_w;
  response->y_w=y_w;
  response->z_w=z_w;
  }

void j_vel(const std::shared_ptr<proj_fk::srv::Jvel::Request> request,
          std::shared_ptr<proj_fk::srv::Jvel::Response>      response)
{
  
  x_v = request->x_v;
  y_v = request->y_v;
  z_v = request->z_v;
 
float J_i[3][3];
 // inverse jacobian
    J_i[0][0] = -(10*cos(q1 + q2))/(9*sin(q2));
    J_i[0][1] = (10*sin(q1 + q2))/(9*sin(q2));
    J_i[0][2] = 0;
    J_i[1][0] = -(10*cos(q1 + q2) + 9*cos(q1))/(9*sin(q2));
    J_i[1][1] = -(10*sin(q1 + q2) + 9*sin(q1))/(9*sin(q2));
    J_i[1][2] = 0;
    J_i[2][0] = 0;
    J_i[2][1] = 0;
    J_i[2][2] = -1;

  //matrix multiplication
  
  j1v = J_i[0][0] * x_v + J_i[0][1] * y_v + J_i[0][2] * z_v;
  j2v = J_i[1][0] * x_v + J_i[1][1] * y_v + J_i[1][2] * z_v;
  j3v = J_i[2][0] * x_v + J_i[2][1] * y_v + J_i[2][2] * z_v;

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f,%f,%f",q1,q2,q3);
  response->j1v=j1v;
  response->j2v=j2v;
  response->j3v=j3v;
   esum[0]=0;
   esum[1]=0;
   esum[2]=0;

} 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    rclcpp::Service<proj_fk::srv::Eevel>::SharedPtr service_ee;

    rclcpp::Service<proj_fk::srv::Jvel>::SharedPtr service_j;

    float q1,q2,q3,c_q1,c_q2,c_q3,c_q_dot1,c_q_dot2,c_q_dot3,j1v,j2v,j3v;
    float old_tme= 0.0;
    float x_v= 0.0,y_v=0.0,z_v = 0.0;
    float Kp=7,Ki=10; 
    float esum[3] = {0,0,0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}
  
