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
  
  Eigen::Vector2f t_minus_1_centroid;
  t_minus_1_centroid << meas_t_minus_1_x_avg, meas_t_minus_1_y_avg;  
  t_minus_1_centroid.x() = t_minus_1_centroid.x()/static_cast<float>(this->msg_t_minus_1.size());
  std::cout << "Minus 1 X Centroid: " << t_minus_1_centroid.x() << std::endl;
  std::cout << "Minus 1 Y Centroid: " << t_minus_1_centroid.y() << std::endl;
  t_minus_1_centroid.y() = t_minus_1_centroid.y()/static_cast<float>(this->msg_t_minus_1.size());

  Eigen::Vector2f t_centroid;
  t_centroid << meas_t_x_avg, meas_t_y_avg;  
  t_centroid.x() = t_minus_1_centroid.x()/static_cast<float>(this->msg_t.size());
  t_centroid.y() = t_minus_1_centroid.y()/static_cast<float>(this->msg_t.size());

 // ROS_INFO("x avg trans: %f", meas_t_x_avg - meas_t_minus_1_x_avg);
 // ROS_INFO("y avg trans: %f", meas_t_y_avg - meas_t_minus_1_y_avg);


  std::vector<Eigen::Vector2f> t_minus_1_prime = this->make_prime_vec(this->msg_t_minus_1, t_minus_1_centroid); 
  std::vector<Eigen::Vector2f> t_prime = this->make_prime_vec(this->msg_t, t_centroid); 

  Eigen::Matrix2f W_SVD;
  KDTree tree;
  tree.buildTree(this->msg_t);

  const float error_threshold = 5; 
  //ISSUE: error doesnt change in while loop?
  float error = 1e9;

  float error_placeholder = 1e10;
  int iter_counter = 0;
  while ( error > error_threshold && error < error_placeholder && iter_counter < 200 ) {
    std::cout << "error 1: " << error <<std::endl;
    error_placeholder = error;
    std::cout << "error_placeholder" <<  error_placeholder  << std::endl;
    float tracking_error = 0;
    for (int i = 0; i < msg_t_minus_1.size(); i++) {
      auto neighbor = tree.nearestNeighbor(msg_t_minus_1[i]);
      tracking_error += neighbor.second;
      W_SVD += t_prime[i] * t_minus_1_prime[i].transpose();
    } 

    error = tracking_error;
    std::cout <<"error: " << error << std::endl;
    iter_counter++;

 }

  std::cout << "W_SVD: " << std::endl << W_SVD <<std::endl;
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(W_SVD , Eigen::ComputeFullU | Eigen::ComputeFullV);
  std::cout << "U_SVD: " << std::endl << svd.matrixU() << std::endl;
  std::cout << "V_SVD: " << std::endl << svd.matrixV() << std::endl;
  Eigen::Matrix2f rotation = svd.matrixU() * svd.matrixV().transpose();
  std::cout << "R: " << std::endl << rotation << std::endl;
  Eigen::Vector2f translation = t_centroid - (rotation * t_minus_1_centroid);




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

