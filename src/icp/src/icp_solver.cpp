#include <math.h>
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "KDTree.h"

class icp {
  public:
    void scanCallback(const sensor_msgs::LaserScan msg);
    Eigen::Matrix3f solveTransform();
    icp() {
      this->enough_msgs=false;
    }

  private:
    //msg 1 is x_t-1 meas
    //msg_2 is x_t meas
    std::vector<Eigen::Vector2f> msg_t_minus_1;
    std::vector<Eigen::Vector2f> msg_t;
    int msg_counter;
    //boolean to check if both msg slots are not empty
    bool enough_msgs;
};


void icp::scanCallback(const sensor_msgs::LaserScan  msg)
{
  this->msg_t_minus_1 = this->msg_t;

  std::vector<Eigen::Vector2f> latest_beam_meas;
  for (int i = 0; i < msg.ranges.size(); i++) {
    //ROS_INFO("Beam [%i]: [%f]", i, msg.ranges[i]);
    // Convert Polar Coordinates to Cartesian:
    float x = msg.ranges[i] * cos(i * (M_PI/180)); 
    float y = msg.ranges[i] * sin(i * (M_PI/180)); 
    Eigen::Vector2f meas(x, y);
    latest_beam_meas.push_back(meas);
    ROS_INFO("Beam [%i]: [%f], [%f]", i, meas[0], meas[1]);
  }
  this->msg_t = latest_beam_meas;

  if (!this->enough_msgs) {
    if(!msg_t_minus_1.size() == 0) {
      this->enough_msgs = true;
    }
  }

  if(this->enough_msgs) {
    this->solveTransform();
  }

    
}

Eigen::Matrix3f icp::solveTransform() {
  //to solve for transformation, we just need to do subtract the averages of the x y coordinates    
  //calc averages
  //  [R R][t]  
  //  [R R][t]
  //  [0 0  1]

  float meas_t_minus_1_x_avg = 0; 
  float meas_t_minus_1_y_avg = 0; 

  for (auto x : this->msg_t_minus_1) {
    meas_t_minus_1_x_avg += x[0];
    meas_t_minus_1_y_avg += x[1];
  }


  float meas_t_x_avg = 0; 
  float meas_t_y_avg = 0; 
  for (auto x : this->msg_t) {
    meas_t_x_avg += x[0];
    meas_t_y_avg += x[1];
  }

  meas_t_minus_1_x_avg /= msg_t_minus_1.size();
  meas_t_minus_1_y_avg /= msg_t_minus_1.size();
  meas_t_x_avg /= msg_t.size();
  meas_t_y_avg /= msg_t.size();

  ROS_INFO("x avg trans: %f", meas_t_x_avg - meas_t_minus_1_x_avg);
  ROS_INFO("y avg trans: %f", meas_t_y_avg - meas_t_minus_1_y_avg);

  // we have to find the rotation transformation
  // we want to take each point from xt-1 and find the nearest neighbor in xt
  // we need to sort out all the points in xt first
  KDTree tree;
  tree.buildTree(this->msg_t);
  
  //Eigen::Vector2f nn_test(.45f, -.67f);
  //tree.nearestNeighbor(nn_test);
  //
  // Although perhaps we don't need to do all that
  //

  std::vector<Eigen::Vector2f> mean_distance_minus_1;
  std::vector<Eigen::Vector2f> mean_distance;

  for (int i = 0; i < msg_t_minus_1.size(); i++) {
    tree.nearestNeighbor(msg_t_minus_1[i]);


  } 

  Eigen::Matrix3f temp_out_delete_later;
  return temp_out_delete_later;

}

//-----------------------------------------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  icp icp_solver;


  ros::init(argc, argv, "icp_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1001, &icp::scanCallback, &icp_solver);
  ros::spin();

  return 0;
}

