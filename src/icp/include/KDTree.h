#include <math.h>
#define _USE_MATH_DEFINES

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

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
  KDNode* nearestNeighbor(Eigen::Vector2f input_pt );

};

float calcDistance(KDNode * node,  Eigen::Vector2f point) {
  return (sqrt(pow(point[0] - node->coordinate[0] ,2) + pow(point[1] - node->coordinate[1], 2)));
}

KDNode* KDTree::buildTree(std::vector<Eigen::Vector2f> point_set, int depth, KDNode * parent_node) {
  //this root_node
  std::cout << "starting node iteration" << std::endl;
  KDNode * node = new KDNode;
  node->left_node=NULL;
  node->right_node=NULL;
  node->parent_node=parent_node;
  node->depth = depth;
  std::cout << "Input Pt set size: " << point_set.size() << std::endl;

  if(point_set.size() <= 1) {
    if(point_set.size() == 1) {
      node->coordinate = point_set[0];

    }

    std::cout << "Reached 1 size" <<std::endl;
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
  std::cout << "median index: " << median_index <<std::endl;
  node->coordinate = point_set[median_index];

  std::vector<Eigen::Vector2f> leftPoints(point_set.begin(), point_set.begin() + median_index);
  std::cout << "Building left node tree" << std::endl;
  node->left_node = buildTree(leftPoints, depth + 1, node);
  std::vector<Eigen::Vector2f> rightPoints(point_set.begin() + median_index + 1, point_set.end());
  std::cout << "Building right node tree" << std::endl;
  node->right_node = buildTree(rightPoints, depth + 1, node);

  this->root_node = node;

  return node;
}

KDNode* KDTree::nearestNeighbor(Eigen::Vector2f input_pt) {
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
        std::cout << "Going to right_node_1!" << std::endl;
      } else {
        if(iter->left_node != NULL) iter = iter->left_node;
        std::cout << "Going to left_node!" << std::endl;
      }
    } else {
      if(input_pt[1] > iter->coordinate[1]) {
        if(iter->right_node != NULL) iter = iter->right_node;
        std::cout << "Going to right_node_2!" << std::endl;

      } else {
        if(iter->left_node != NULL) iter = iter->left_node;
        std::cout << "Going to left_node!" << std::endl;
      }
    }

  }

  std::cout << "nearest neighbor coordinate: " << nearest_neighbor->coordinate[0] << ", " << nearest_neighbor->coordinate[1] << std::endl;
  //the above should return us the leaf node associated with this input pt
  return nearest_neighbor;

}

