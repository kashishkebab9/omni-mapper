#include "ros/ros.h"
#include <cmath>
#include <gazebo_msgs/GetLinkState.h>
#include <stdlib.h>     /* abs */
#include <iostream>
#include <ekf_implementation/encoder.h>
#include <math.h>
#define _USE_MATH_DEFINES

class tb_sim_enc {
  public:
    tb_sim_enc() {
      this->prev_left = 0.0;
      this->prev_right = 0.0;
      this->prev_timestamp = ros::Time::now();
      ROS_INFO("Starting GetLinkState Service Client...");
      ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

      this->encoder_pub = nh.advertise<ekf_implementation::encoder>("encoder_ticks", 1000);
      gazebo_msgs::GetLinkState left_srv;
      gazebo_msgs::GetLinkState right_srv;

      left_srv.request.link_name = "wheel_left_link";
      right_srv.request.link_name = "wheel_right_link";
      left_srv.request.reference_frame = "base_footprint";
      right_srv.request.reference_frame = "base_footprint";

      int frequency_to_pub_enc = 10; 
      ros::Rate rate(frequency_to_pub_enc);

      while(ros::ok()) {
        if (client.call(left_srv) && client.call(right_srv))
        {

          ros::Time msg_ts = ros::Time::now();
          
          //rad/s for each wheel
          float left_wheel_ang_vel = (float)left_srv.response.link_state.twist.angular.y;
          float right_wheel_ang_vel = (float)right_srv.response.link_state.twist.angular.y;

          ROS_INFO("Left Val: %f", left_wheel_ang_vel);
          ROS_INFO("Right Val: %f", right_wheel_ang_vel);

          ros::Duration delta_t = msg_ts - this->prev_timestamp; 
          float delta_t_float = delta_t.toSec();
          ROS_INFO("delta_t: %f", delta_t_float); 

          auto rad_travelled_left = (this->prev_left * delta_t_float) + (left_wheel_ang_vel - this->prev_left)* delta_t_float/2;
          auto rad_travelled_right = (this->prev_right * delta_t_float) + (right_wheel_ang_vel - this->prev_right)* delta_t_float/2;
          float deg_travelled_left = (rad_travelled_left * 180/M_PI);
          float deg_travelled_right = (rad_travelled_right * 180/M_PI);

          ROS_INFO("left radians travelled: %f radians", rad_travelled_left); 
          ROS_INFO("right radians travelled: %f radians", rad_travelled_right); 

          ROS_INFO("left encoder: %f ticks", floor(deg_travelled_left)); 
          ROS_INFO("right encoder: %f ticks", floor(deg_travelled_right)); 

          ekf_implementation::encoder encoder_msg;
          encoder_msg.right_encoder = floor(deg_travelled_right);
          encoder_msg.left_encoder = floor(deg_travelled_left);
          encoder_pub.publish(encoder_msg);

          this->prev_left = left_wheel_ang_vel;
          this->prev_right = right_wheel_ang_vel;
          this->prev_timestamp = msg_ts;

        } else {
          ROS_ERROR("Failed to call wheel link srv");
        }
        rate.sleep();
        std::cout << ros::Time::now() <<std::endl;
      }
    }

  private:
    ros::NodeHandle nh, pnh;
    ros::Publisher encoder_pub;
    float prev_left;
    float prev_right;
    ros::Time prev_timestamp;
    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulated_encoder");
  tb_sim_enc solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
