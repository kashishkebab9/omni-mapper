#include <math.h>
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

struct KDNode {
  public:
    KDNode * left_node;
    KDNode * right_node;
    Eigen::Vector2f coordinate;
    int depth;
};

KDNode* buildKDTree(std::vector<Eigen::Vector2f> point_set, int depth) {
  KDNode * node = new KDNode;
  node->left_node = nullptr;
  node->right_node = nullptr;
  node->depth = depth;


  if (point_set.size() == 1) {
    return node;
  }

  if ( depth % 2 == 0 ) {
    std::sort(point_set.begin(), point_set.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
      return a.x() < b.x();
    });
  } else {
    std::sort(point_set.begin(), point_set.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
      return a.y() < b.y();
    });
  }

  int median_index = point_set.size()/2;
  node->coordinate = point_set[median_index];

  std::vector<Eigen::Vector2f> leftPoints(point_set.begin(), point_set.begin() + median_index);
  node->left_node = buildKDTree(leftPoints, depth + 1);

  std::vector<Eigen::Vector2f> rightPoints(point_set.begin() + median_index + 1, point_set.end());
  node->right_node = buildKDTree(rightPoints, depth + 1);

  return node;
}

KDNode* NearestNeigborKD(KDNode* node, Eigen::Vector2f input_pt) {

  //A Nearest Neighbor Method for Efficient ICP
  //   o    x  is input x greater than o.x?
  //  o o   y  is input y greater than o.y? 
  // o o o  x  is input x greater than o.x?

  KDNode * iter;
  iter = node;

  //Deal with bullshit unputs:
  if (node == nullptr) {
    ROS_INFO("Input KDTree Node is NULL!"); 
    return node;
  }
  
  if(node->left_node == nullptr && node->right_node == nullptr) {
    //we have reached leaf and must start comparing upwards
    auto leaf_distance = sqrt(pow(input_pt[0] - iter->coordinate[0], 2) + pow(input_pt[1] - iter->coordinate[1],2)) ;
  }

  int depth = node->depth;

  if (depth %2 == 0) {
    //we want to compare x values of input and node
    if(input_pt[0] > node->coordinate[0]) {
      //next node is right child of this one
      iter = node->right_node;
    } else {
      //next node is left child of this one
      iter = node->left_node;
    }
  } else {
    //we want to compare y values of input and node
    if(input_pt[1] > node->coordinate[1]) {
      //next node is right child of this one
      iter=node->right_node;
    } else {
      //next node is left child of this one
      iter = node->left_node;
    }
  }

  NearestNeigborKD(node, input_pt);

  
}


//-----------------------------------------------------------------------------------------------------------//

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

  std::cout << "x avg trans: " <<  meas_t_x_avg - meas_t_minus_1_x_avg << std::endl;
  std::cout << "y avg trans: " <<  meas_t_y_avg - meas_t_minus_1_y_avg << std::endl;

  // we have to find the rotation transformation
  // we want to perform kd-tree nearest neighbor search
  // we want to take each point from xt-1 and find the nearest neighbor in xt
  // we need to sort out all the points in xt first

  buildKDTree(this->msg_t,  0);
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

