#include "ros/ros.h"
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>

class motion_model {
  public:
    motion_model() {
      this->pose_estimate << 0, 0, 0;
      command_sub = this->nh.subscribe("goal_pose", 1, &motion_model::command_callback, this);
      pose_estimate_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 1);
    }

    void command_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  private:

    ros::NodeHandle nh, pnh;
    ros::Subscriber command_sub;
    ros::Publisher pose_estimate_pub;
    Eigen::Vector3f pose_estimate;
};

void motion_model::command_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3f goal_pose;
  goal_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z;

  std::cout << "received goal "    << std::endl << goal_pose.matrix()           <<  std::endl;
  std::cout << "current estimate " << std::endl << this->pose_estimate.matrix() << std::endl;

  float d_rot_1 = atan2(msg->pose.position.y - this->pose_estimate.y(), msg->pose.position.x - this->pose_estimate.x());  
  float d_trans = pow(pow(goal_pose.x() - this->pose_estimate.x(), 2) + pow(goal_pose.y() - this->pose_estimate.y(), 2), .5) ;
  float d_rot_2 = (goal_pose[2] - this->pose_estimate[2]) - d_rot_1;

  std::cout << "Commands are decomposed into: " << std::endl;
  std::cout << "Rotation 1: "  << d_rot_1 << std::endl;
  std::cout << "Translation: " << d_trans << std::endl;
  std::cout << "Rotation 2: "  << d_rot_2 << std::endl;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_model");
  motion_model solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
