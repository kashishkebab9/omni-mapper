#include <math.h>
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <experimental/optional>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "KDTree.h"

//TODO: Need to debug the kd tree
//TODO: need to use kd tree as supervisor for loop transform

class icp {
  public:

    icp() {
      this->enough_msgs=false;
    }

    void scanCallback(const sensor_msgs::LaserScan msg);
    Eigen::Matrix3f solve_transformation_loop(std::vector<Eigen::Vector3f> prev_scan, std::vector<Eigen::Vector3f> current_scan);
    Eigen::Vector2f calculateCentroid(std::vector<Eigen::Vector3f> input_set);
  
  private:
    //msg 1 is x_t-1 meas
    //msg_2 is x_t meas
    std::vector<Eigen::Vector3f> msg_t_minus_1;
    std::vector<Eigen::Vector3f> msg_t;
    std::vector<Eigen::Vector2f> make_prime_vec(std::vector<Eigen::Vector3f> msg, Eigen::Vector2f avg);
    //boolean to check if both msg slots are not empty
    bool enough_msgs;
};


void icp::scanCallback(const sensor_msgs::LaserScan  msg)
{
  this->msg_t_minus_1 = this->msg_t;

  std::vector<Eigen::Vector3f> latest_beam_meas;
  for (int i = 0; i < msg.ranges.size(); i++) {
    // Convert Polar Coordinates to Cartesian:
    if (abs(msg.ranges[i]) > .01) {
      float x = msg.ranges[i] * cos(i * (M_PI/180)); 
      float y = msg.ranges[i] * sin(i * (M_PI/180)); 
      Eigen::Vector3f meas(x, y, 1);
      latest_beam_meas.push_back(meas);
      ROS_DEBUG("Beam [%i]: [%f], [%f], [%f]", i, meas[0], meas[1], meas[2]);
    }
  }
  this->msg_t = latest_beam_meas;
  if (!this->enough_msgs) {
    if(!msg_t_minus_1.size() == 0) {
      this->enough_msgs = true;
    }
  }

  //TODO: modify this to for actual iteration
  if(this->enough_msgs) {

    this->solve_transformation_loop(this->msg_t_minus_1, this->msg_t);
  }

}

Eigen::Vector2f icp::calculateCentroid(std::vector<Eigen::Vector3f> input_set) {
  float avg_x = 0;
  float avg_y = 0;

  for (auto i: input_set) {
    if (i.x() != 0 && i.y() != 0 && i.x() !=-0 && i.y() != 0 ) {
      avg_x += i.x(); 
      avg_y += i.y();
    }
  }

  avg_x = avg_x/static_cast<float>(input_set.size());
  avg_y = avg_y/static_cast<float>(input_set.size());

  Eigen::Vector2f output;
  output << avg_x, avg_y;
  std::cout << "centroid: " << output <<std::endl;
  return output;
}


//********************************************************************************//
Eigen::Matrix3f icp::solve_transformation_loop(std::vector<Eigen::Vector3f> prev_scan, std::vector<Eigen::Vector3f> current_scan) {
  //we need to calculate the centroids:
  Eigen::Vector2f prev_centroid = this->calculateCentroid(prev_scan);
  Eigen::Vector2f current_centroid = this->calculateCentroid(current_scan);

  //we need to generate variance prime vec:
  std::vector<Eigen::Vector2f> prev_prime = this->make_prime_vec(prev_scan,  prev_centroid); 
  std::vector<Eigen::Vector2f> current_prime = this->make_prime_vec(current_scan,  current_centroid); 

  //we need to start setting up our SVD:
  size_t summation_iteration;
  if (prev_scan.size() > current_scan.size()) {
    summation_iteration = current_scan.size();
  } else {
    summation_iteration = prev_scan.size();
  }

  Eigen::Matrix2f W_final = Eigen::Matrix2f::Zero();
  for (size_t i= 0; i < summation_iteration ; i++) {
    Eigen::Matrix2f w_loop = current_prime[i] * prev_prime[i].transpose();
    W_final += w_loop;
  }

  //perform mentioned SVD:
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(W_final, Eigen::ComputeFullU | Eigen::ComputeFullV);
  std::cout << "U: " << std::endl << svd.matrixU() << std::endl;
  std::cout << "V: " << std::endl << svd.matrixV() << std::endl;
  
  //get our transformation Matrix:
  Eigen::Matrix2f rotation = svd.matrixU() * svd.matrixV();
  Eigen::Vector2f translation = current_centroid - (rotation * prev_centroid);
  Eigen::Matrix3f transformation;
  transformation.setIdentity();
  transformation.block<2,2>(0,0) = rotation;
  transformation.block<2,1>(0,2) = translation;
  std::cout << "Transformation: " << std::endl << transformation.matrix() << std::endl;
  return transformation;
}

std::vector<Eigen::Vector2f> icp::make_prime_vec(std::vector<Eigen::Vector3f> msg, Eigen::Vector2f input_centroid) {
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
