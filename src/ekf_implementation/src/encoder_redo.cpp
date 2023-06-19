#include "ros/ros.h"
#include <cmath>
#include <cstddef>
#include <gazebo_msgs/LinkStates.h>
#include <stdlib.h>     /* abs */
#include <iostream>

class tb_sim_enc {
  public:
    tb_sim_enc() {
      this->prev_left = 0.0;
      this->prev_right = 0.0;
      this->encoder_res = 365; // one tick per degree of rotation
      this->min_range = -.707386;
      this->total_range = min_range * -2;

      this->link_states_sub = this->nh.subscribe("/gazebo/link_states", 1, &tb_sim_enc::link_callback, this);

    }

    void link_callback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
      std::cout  << "Received msg" << std::endl;
      bool has_left_wheel = false;
      bool has_right_wheel = false;
      size_t left_ind = 0;
      size_t right_ind = 0;
      for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "turtlebot3::wheel_left_link") {
          has_left_wheel = true;
          left_ind = i;
        }
        if (msg->name[i] == "turtlebot3::wheel_right_link") {
          has_right_wheel = true;
          right_ind = i;
        }
      }

      if (!has_left_wheel || !has_right_wheel) {
        std::cout << "We don't have one of the wheel states!" << std::endl;
        return;
      }

      float left_val = msg->pose[left_ind].orientation.z;
      float right_val = msg->pose[right_ind].orientation.z;
      float asin_left = std::asin(left_val);
      float asin_right = std::asin(right_val);
      std::cout << "asin_left: " << asin_left << std::endl;
      std::cout << "asin_right: " << asin_right << std::endl;

      float left_diff = std::abs(left_val - this->prev_left);
      float left_fraction = (left_diff)/total_range;
      float left_encoder = (left_fraction * this->encoder_res);
      left_encoder = floor(left_encoder);

      float right_diff = std::abs(right_val - this->prev_right);
      float right_fraction = (right_diff)/total_range;
      float right_encoder = (right_fraction * this->encoder_res);
      right_encoder = floor(right_encoder);

      ROS_INFO("Prev Val: %f", this->prev_left);
      ROS_INFO("Returned: %f", left_val);
      ROS_INFO("left_diff: %f", left_diff);
      ROS_INFO("left_fraction: %f", left_fraction);
      ROS_INFO("left_encoder_ticks: %f", left_encoder);
      ROS_INFO("\n");
      ROS_INFO("Prev Val: %f", this->prev_right);
      ROS_INFO("Returned: %f", right_val);
      ROS_INFO("right_diff: %f", right_diff);
      ROS_INFO("right_fraction: %f", right_fraction);
      ROS_INFO("right_encoder_ticks: %f", right_encoder);
      ROS_INFO("\n");

      this->prev_left = left_val;
      this->prev_right = right_val;
    }

  private:
    ros::NodeHandle nh, pnh;
    ros::Subscriber link_states_sub;
    float prev_left;
    float prev_right;
    float min_range;
    float total_range;
    int encoder_res;
    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulated_encoder");
  tb_sim_enc solver;

  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
