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

class icp {
  public:

    icp() {
      this->enough_msgs=false;
    }

    void scanCallback(const sensor_msgs::LaserScan msg);
    std::experimental::optional<Eigen::Matrix3f> solveTransform();
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
    std::cout << "msg_t x:" << std::endl;
    for (int i = 0; i < this->msg_t.size(); i++) {
      std::cout << msg_t[i].x() << ", ";
    }
    std::cout <<std::endl;
    std::cout << "msg_t y:" << std::endl;
    for (int i = 0; i < this->msg_t.size(); i++) {
      std::cout << msg_t[i].y() << ", ";
    }
    std::cout <<std::endl;

    std::cout << "msg_t_minus_1 x:" << std::endl;
    for (int i = 0; i < this->msg_t_minus_1.size(); i++) {
      std::cout << msg_t_minus_1[i].x() << ", ";
    }
    std::cout <<std::endl;
    std::cout << "msg_t_minus_1 y:" << std::endl;
    for (int i = 0; i < this->msg_t_minus_1.size(); i++) {
      std::cout << msg_t_minus_1[i].y() << ", ";
    }
    std::cout <<std::endl;
    this->solveTransform();
  }
}

Eigen::Vector2f icp::calculateCentroid(std::vector<Eigen::Vector2f> input_set) {
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
  return output;
}


//********************************************************************************//
std::experimental::optional<Eigen::Matrix3f> icp::solveTransform() {
// msg_t operations:  
  Eigen::Vector2f t_centroid = this->calculateCentroid(this->msg_t);
  KDTree tree;
  tree.buildTree(this->msg_t);
  std::vector<Eigen::Vector2f> t_prime = this->make_prime_vec(this->msg_t, t_centroid); 

// msg_t-1 operations:  
  Eigen::Vector2f t_minus_1_centroid = this->calculateCentroid(this->msg_t_minus_1);
  std::vector<Eigen::Vector2f> t_minus_1_prime = this->make_prime_vec(this->msg_t_minus_1, t_minus_1_centroid); 

  std::cout << "t_minus_1_centroid: " << t_minus_1_centroid << std::endl;
  std::cout << "t_centoid: " << t_centroid << std::endl;
  float error = 0;

  Eigen::Matrix2f W_SVD;
  for (int i = 0; i < msg_t_minus_1.size(); i++) {
    auto neighbor = tree.nearestNeighbor(msg_t_minus_1[i]);
    //pt to pt distance:
    error += neighbor.second;

    // 2 by 1 times 1 by 2 == 2 by 2
    W_SVD += t_prime[i] * t_minus_1_prime[i].transpose();
  } 

  std::cout << "error: " << error << std::endl;
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(W_SVD , Eigen::ComputeFullU | Eigen::ComputeFullV);

  std::cout << "Initial W_SVD: " << std::endl << W_SVD <<std::endl;
  std::cout << "Initial U_SVD: " << std::endl << svd.matrixU() << std::endl;
  std::cout << "Initial V_SVD: " << std::endl << svd.matrixV() << std::endl;

  Eigen::Matrix2f rotation = svd.matrixU() * svd.matrixV().transpose();
  // 2 by 2 times 2 by 1  == 2 by 1
  // 2 by 1 minus 2 by 1
  Eigen::Vector2f translation = t_centroid - (rotation * t_minus_1_centroid);
  Eigen::Matrix3f transformation;
  transformation.setIdentity();
  transformation.block<2,2>(0,0) = rotation;
  std::cout << "translation: " << translation << std::endl;
  transformation.block<2,1>(0,2) = translation;

  //we need to apply this tranformation to the msg_t_minus_1:
  std::vector<Eigen::Vector2f> transformed_msg_iteration;
  Eigen::Matrix3f final_transformation; 
  size_t neighbor_size;
  if (this->msg_t_minus_1.size() > this->msg_t.size()) {
    neighbor_size = this->msg_t.size();
  } else {
    neighbor_size = this->msg_t_minus_1.size();
  }
  for (size_t i = 0; i < neighbor_size; i++) {
//    std::cout << "point number: " << i << std::endl;
//    std::cout << "point x: " << msg_t_minus_1[i].x() << std::endl;
//    std::cout << "point y: " << msg_t_minus_1[i].y() << std::endl;
    //formulate the homogenous version of the ith 2d vector in msg_t_minus_1:
    Eigen::Vector3f homogenous_3f;
    homogenous_3f << msg_t_minus_1[i].x(), msg_t_minus_1[i].y(), 1;
    //apply the transform:
    Eigen::Vector3f transformed_vec;
    transformed_vec = transformation * homogenous_3f; 
    Eigen::Vector2f transformed_vec_2f;
    transformed_vec_2f << transformed_vec.x(), transformed_vec.y();
//    std::cout << "transformed x: " << transformed_vec.x() << std::endl;
//    std::cout << "transformed y: " << transformed_vec.y() << std::endl;
    transformed_msg_iteration.push_back(transformed_vec_2f);
  }
  final_transformation = transformation;

//********************************************************************************//
  float error_threshold = 3.5;
  float error_placeholder = error+ 1.0;
  int iteration_counter =0;

  while (error > error_threshold && error <= error_placeholder && iteration_counter < 10 ) {
    ROS_INFO("In the while loop!");
    std::cout << "iteration count: " << iteration_counter <<std::endl;
    std::cout << "error: " << error << std::endl; 
    std::cout << "error_placeholder: " << error_placeholder << std::endl; 

    error_placeholder = error;
    Eigen::Vector2f transformed_centroid = this->calculateCentroid(transformed_msg_iteration);
    std::vector<Eigen::Vector2f> transformed_prime = this->make_prime_vec(transformed_msg_iteration, transformed_centroid); 

    Eigen::Matrix2f W_SVD_loop;

    error = 0;
    size_t neighbor_index_size;

    if (transformed_msg_iteration.size() > this->msg_t.size()) {
      neighbor_index_size = this->msg_t.size();
    } else {
      neighbor_index_size = transformed_msg_iteration.size();
    }

    for (int i = 0; i < neighbor_index_size; i++) {
      auto neighbor = tree.nearestNeighbor(transformed_msg_iteration[i]);
      error += neighbor.second;
      W_SVD_loop += t_prime[i] * transformed_prime[i].transpose();
    } 
    std::cout << "error after tree distance for loop: " << error << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix2f> svd_loop(W_SVD_loop , Eigen::ComputeFullU | Eigen::ComputeFullV);

    std::cout << "W_SVD: " << std::endl << W_SVD_loop <<std::endl;
    std::cout << "U_SVD: " << std::endl << svd_loop.matrixU() << std::endl;
    std::cout << "V_SVD: " << std::endl << svd_loop.matrixV() << std::endl;

    Eigen::Matrix2f rotation_loop = svd_loop.matrixU() * svd_loop.matrixV().transpose();
    Eigen::Vector2f translation_loop = t_centroid - (rotation_loop * t_minus_1_centroid);
    Eigen::Matrix3f transformation_loop;
    transformation_loop.setIdentity();
    transformation_loop.block<2,2>(0,0) = rotation_loop;
    transformation_loop.block<1,2>(2,0) = translation_loop;

    //we need to apply this tranformation to the msg_t_minus_1:
    std::vector<Eigen::Vector2f> transformed_msg_iteration_placeholder;
    for (size_t i = 0; i < transformed_msg_iteration.size(); i++) {
      //formulate the homogenous version of the ith 2d vector in msg_t_minus_1:
      Eigen::Vector3f homogenous_3f;
      homogenous_3f << transformed_msg_iteration[i].x(), transformed_msg_iteration[i].y(), 1;
      //apply the transform:
      Eigen::Vector3f transformed_vec;
      transformed_vec = transformation_loop * homogenous_3f; 
      Eigen::Vector2f transformed_vec_2f; 
      transformed_vec_2f << transformed_vec.x(), transformed_vec.y();
      transformed_msg_iteration_placeholder.push_back(transformed_vec_2f);
    }

    final_transformation = transformation_loop * final_transformation;

    transformed_msg_iteration.clear();
    transformed_msg_iteration = transformed_msg_iteration_placeholder;
    transformed_msg_iteration_placeholder.clear();
    iteration_counter++;
    std::cout << "end error: " << error << std::endl;
  }

  if (error < 9.0) {
    std::cout << "Rotation: " << acos(final_transformation(0,0));
    std::cout << "final transform: " << std::endl << final_transformation << std::endl;
    return final_transformation;
  } 
  ROS_WARN("Error is too high! Throwing away this dataset");
}

std::vector<Eigen::Vector2f> icp::make_prime_vec(std::vector<Eigen::Vector2f> msg, Eigen::Vector2f input_centroid) {
  std::vector<Eigen::Vector2f> prime_vec;
  ROS_INFO("Making Prime Vec");
  for(size_t i = 0; i < msg.size(); i++) {
    if (msg[i].x() != 0 && msg[i].y() != -0 && msg[i].y() != 0 && msg[i].y() != -0) {
      float x = (msg[i]).x() - input_centroid.x();
      float y = (msg[i]).y() - input_centroid.y();

      Eigen::Vector2f prime_element;
      prime_element << x, y;
      prime_vec.push_back(prime_element);
    }
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

