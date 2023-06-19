#include "ros/ros.h"
#include <boost/bind/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <ekf_implementation/encoder.h>
#include <math.h>
#define _USE_MATH_DEFINES

class ekf {
  public:
    ekf() {

      this->wheel_diameter = .066; // m
      this->encoder_res = 360;
      this->dw = .144; // m

      pose_estimate << 0, 0, 0;
      std::cout << "Pose estimate: " << pose_estimate.matrix() << std::endl;
      encoder_sub = this->nh.subscribe("/encoder_ticks", 1, &ekf::encoder_callback, this);
      laser_odom_sub = this->nh.subscribe("/scarab41/odom_laser", 1, &ekf::laser_callback, this);

    }

    void encoder_callback(const ekf_implementation::encoder::ConstPtr& msg);
    void laser_callback(const nav_msgs::Odometry::ConstPtr& msg);


  private:

    ros::NodeHandle nh, pnh;
    ros::Subscriber encoder_sub;
    ros::Subscriber laser_odom_sub;
    Eigen::Vector3f pose_estimate;
    float wheel_diameter;
    int encoder_res;
    float dw; //distance between tires/2
    
};

void ekf::encoder_callback(const ekf_implementation::encoder::ConstPtr& msg) {

  ROS_INFO("Received Encoder Reading!");
  ROS_INFO("Right: %i", msg->right_encoder);
  ROS_INFO("Left: %i", msg->left_encoder);

  float right_dist = msg->right_encoder * this->wheel_diameter * M_PI/(this->encoder_res);
  float left_dist = msg->left_encoder * this->wheel_diameter * M_PI/(this->encoder_res);

  ROS_INFO("Distance Travelled by Right wheel: %f m", right_dist);
  ROS_INFO("Distance Travelled by Left wheel: %f m", left_dist);

  float delta_theta = (right_dist - left_dist)/(this->dw*2);
  float delta_trans = (right_dist + left_dist)/2;

  ROS_INFO("Delta Theta: %f m", delta_theta);
  ROS_INFO("Delta Trans: %f m", delta_trans);

  float x_estimate = this->pose_estimate.x() + delta_trans * cos(this->pose_estimate[2] + delta_theta/2) ;
  float y_estimate = this->pose_estimate.y() + delta_trans * sin(this->pose_estimate[2] + delta_theta/2) ;
  float theta_estimate = this->pose_estimate[2] + delta_theta;
  while(theta_estimate > M_PI * 2) {
    theta_estimate = theta_estimate - 2 * M_PI;
  }

  ROS_INFO("x estimate: %f m", x_estimate);
  ROS_INFO("y estimate: %f m", y_estimate);
  ROS_INFO("theta estimate: %f m", theta_estimate);

  this->pose_estimate << x_estimate, y_estimate, theta_estimate;
}

void ekf::laser_callback(const nav_msgs::Odometry::ConstPtr& msg) {

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf");
  ekf solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
