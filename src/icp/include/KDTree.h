#include <math.h>

#define _USE_MATH_DEFINES
//#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <utility>

struct KDNode {
  public:
    KDNode * left_node;
    KDNode * right_node;
    KDNode * parent_node;
    Eigen::Vector2f coordinate;
    int depth;
};

class KDTree {
  public:
  KDNode* root_node;
  KDNode* buildTree(std::vector<Eigen::Vector2f> point_set, int depth=0, KDNode * parent_node=NULL);
  std::pair<KDNode*, float> nearestNeighbor(Eigen::Vector2f input_pt );

};

float calcDistance(KDNode * node,  Eigen::Vector2f point) {
  return (sqrt(pow(point[0] - node->coordinate[0] ,2) + pow(point[1] - node->coordinate[1], 2)));
}

KDNode* KDTree::buildTree(std::vector<Eigen::Vector2f> point_set, int depth, KDNode * parent_node) {
  //this root_node
  KDNode * node = new KDNode;
  node->left_node=NULL;
  node->right_node=NULL;
  node->parent_node=parent_node;
  node->depth = depth;

  if(point_set.size() <= 1) {
    if(point_set.size() == 1) {
      node->coordinate = point_set[0];

    }

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
  node->left_node = buildTree(leftPoints, depth + 1, node);
  std::vector<Eigen::Vector2f> rightPoints(point_set.begin() + median_index + 1, point_set.end());
  node->right_node = buildTree(rightPoints, depth + 1, node);

  this->root_node = node;

  return node;
}

std::pair<KDNode*, float> KDTree::nearestNeighbor(Eigen::Vector2f input_pt) {
 // ROS_DEBUG("Input Point: %f, %f", input_pt[0], input_pt[1]);
//  std::cout << "Input Point: " << input_pt[0] << ", " << input_pt[1] << std::endl;
  KDNode * iter = this->root_node; 
  KDNode * nearest_neighbor;

  float best_dist = 1000000.0;
  //as we traverse down we want to record the nearest_neighbor we meet
  while (iter->right_node != nullptr && iter->left_node !=nullptr) {
    
    float dist = calcDistance(iter, input_pt);
    if(dist < best_dist){
      best_dist = dist; 
      nearest_neighbor = iter;
    }

    if (iter->depth % 2 == 0) {
      if(input_pt[0] > iter->coordinate[0]) {
        if(iter->right_node != NULL) iter = iter->right_node;
      } else {
        if(iter->left_node != NULL) iter = iter->left_node;
      }
    } else {
      if(input_pt[1] > iter->coordinate[1]) {
        if(iter->right_node != NULL) iter = iter->right_node;

      } else {
        if(iter->left_node != NULL) iter = iter->left_node;
      }
    }
  }
  //we need to unwind and check recursively for any nearer neighbors    
  iter = iter->parent_node;
  while (iter != this->root_node) {
    iter = iter->parent_node;
    float right_child_dist = calcDistance(iter->right_node, input_pt);
    float left_child_dist = calcDistance(iter->left_node, input_pt);
    if (right_child_dist < best_dist) {
      best_dist = right_child_dist;
      nearest_neighbor = iter->right_node;
    }
    if (left_child_dist < best_dist) {
      best_dist = left_child_dist;
      nearest_neighbor = iter->left_node;
    }
  }

 // ROS_DEBUG("Nearest Neighbor Coordinate: [%f, %f]", nearest_neighbor->coordinate[0], nearest_neighbor->coordinate[1]);
//  std::cout << "Nearest Neighbor Coordinate: " << nearest_neighbor->coordinate[0] << ", " << nearest_neighbor->coordinate[1] << std::endl; 
//  ROS_DEBUG("Distance: %f", best_dist);
  std::cout << "Distance: " << best_dist << std::endl;
  //the above should return us the leaf node associated with this input pt
  auto output = std::make_pair(nearest_neighbor, best_dist);
  return output; 

}

