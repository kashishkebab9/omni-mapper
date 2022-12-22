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

    icp() {
      this->enough_msgs=false;
    }

    void scanCallback(const sensor_msgs::LaserScan msg);
    Eigen::Matrix3f solveTransform();
    Eigen::Vector2f calculateCentroid(std::vector<Eigen::Vector2f> input_set);
    


  private:
    //msg 1 is x_t-1 meas
    //msg_2 is x_t meas
    std::vector<Eigen::Vector2f> msg_t_minus_1;
    std::vector<Eigen::Vector2f> msg_t;
    std::vector<Eigen::Vector2f> make_prime_vec(std::vector<Eigen::Vector2f> msg, Eigen::Vector2f avg);
    //boolean to check if both msg slots are not empty
    bool enough_msgs;
};


void icp::scanCallback(const sensor_msgs::LaserScan  msg)
{
  this->msg_t_minus_1 = this->msg_t;

  std::vector<Eigen::Vector2f> latest_beam_meas;
  for (int i = 0; i < msg.ranges.size(); i++) {
    // Convert Polar Coordinates to Cartesian:
    float x = msg.ranges[i] * cos(i * (M_PI/180)); 
    float y = msg.ranges[i] * sin(i * (M_PI/180)); 
    Eigen::Vector2f meas(x, y);
    latest_beam_meas.push_back(meas);
    ROS_DEBUG("Beam [%i]: [%f], [%f]", i, meas[0], meas[1]);
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

Eigen::Vector2f icp::calculateCentroid(std::vector<Eigen::Vector2f> input_set) {
  float avg_x = 0;
  float avg_y = 0;

  for (auto i: input_set) {
    avg_x += i.x(); 
    avg_y += i.y();
  }

  avg_x = avg_x/static_cast<float>(input_set.size());
  avg_y = avg_y/static_cast<float>(input_set.size());

  Eigen::Vector2f output;
  output << avg_x, avg_y;
  return output;
}

Eigen::Matrix3f icp::solveTransform() {
//********************************************************************************//
// need to calculate the centroids of both msgs
  
  Eigen::Vector2f t_minus_1_centroid = this->calculateCentroid(this->msg_t_minus_1);
  Eigen::Vector2f t_centroid = this->calculateCentroid(this->msg_t);

//********************************************************************************//
// need to find the prime vectors (variance in the matrix) and the nearest neighbors

  std::vector<Eigen::Vector2f> t_minus_1_prime = this->make_prime_vec(this->msg_t_minus_1, t_minus_1_centroid); 
  std::vector<Eigen::Vector2f> t_prime = this->make_prime_vec(this->msg_t, t_centroid); 

  Eigen::Matrix2f W_SVD;
  KDTree tree;
  tree.buildTree(this->msg_t);

//********************************************************************************//
//we need to have an initial solution first to loop through

  float initial_error = 0;
  for (int i = 0; i < msg_t_minus_1.size(); i++) {
    auto neighbor = tree.nearestNeighbor(msg_t_minus_1[i]);
    initial_error += neighbor.second;
    W_SVD += t_prime[i] * t_minus_1_prime[i].transpose();
  } 
  float error = initial_error;

  //just to verify we are not crazy:
  std::cout << "initial_error variable: " << initial_error << std::endl;
  std::cout << "error variable: " << error << std::endl;

  Eigen::JacobiSVD<Eigen::Matrix2f> svd(W_SVD , Eigen::ComputeFullU | Eigen::ComputeFullV);

  std::cout << "Initial W_SVD: " << std::endl << W_SVD <<std::endl;
  std::cout << "Initial U_SVD: " << std::endl << svd.matrixU() << std::endl;
  std::cout << "Initial V_SVD: " << std::endl << svd.matrixV() << std::endl;

  Eigen::Matrix2f rotation = svd.matrixU() * svd.matrixV().transpose();
  Eigen::Vector2f translation = t_centroid - (rotation * t_minus_1_centroid);
  Eigen::Matrix3f transformation;
  transformation.setIdentity();
  transformation.block<2,2>(0,0) = rotation;
  transformation.block<2,1>(2,0) = translation;

  //we need to apply this tranformation to the msg_t_minus_1:
  std::vector<Eigen::Vector3f> transformed_msg_t_minus_1;
  for (size_t i = 0; i < msg_t_minus_1.size(); i++) {
    //formulate the homogenous version of the ith 2d vector in msg_t_minus_1:
    Eigen::Vector3f homogenous_2f;
    homogenous_2f << msg_t_minus_1[i].x(), msg_t_minus_1[i].y(), 1;
    //apply the transform:
    Eigen::Vector3f transformed_vec;
    transformed_vec = transformation * homogenous_2f; 
    transformed_msg_t_minus_1.push_back(transformed_vec);
  }


  Eigen::Matrix3f temp_out_delete_later;
  return temp_out_delete_later;
}

std::vector<Eigen::Vector2f> icp::make_prime_vec(std::vector<Eigen::Vector2f> msg, Eigen::Vector2f input_centroid) {
  std::vector<Eigen::Vector2f> prime_vec;
  ROS_INFO("Making Prime Vec");
  for(size_t i = 0; i < msg.size(); i++) {
    float x = (msg[i]).x() - input_centroid.x();
    float y = (msg[i]).y() - input_centroid.y();

    Eigen::Vector2f prime_element;
    prime_element << x, y;
    prime_vec.push_back(prime_element);
  } 
  return prime_vec;
}

int main(int argc, char **argv)
{
  icp icp_solver;
  ros::init(argc, argv, "icp_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1001, &icp::scanCallback, &icp_solver);
  ros::spin();

  return 0;
}

