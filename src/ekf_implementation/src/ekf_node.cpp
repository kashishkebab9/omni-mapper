#include "ros/ros.h"
#include <boost/bind/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>

class ekf {
  public:
    ekf() {
      pose_estimate << 0, 0, 0;
      motor_odom_sub = this->nh.subscribe("/scarab41/odom_motor", 1, &ekf::motor_callback, this);
      laser_odom_sub = this->nh.subscribe("/scarab41/odom_laser", 1, &ekf::laser_callback, this);

    }

    void motor_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void laser_callback(const nav_msgs::Odometry::ConstPtr& msg);


  private:

    ros::NodeHandle nh, pnh;
    ros::Subscriber motor_odom_sub;
    ros::Subscriber laser_odom_sub;
    Eigen::Vector3f pose_estimate;
    
};

void ekf::motor_callback(const nav_msgs::Odometry::ConstPtr& msg) {

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
