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
  KDNode* buildTree(std::vector<Eigen::Vector2f> point_set, int depth=0, KDNode * parent_node=NULL);

};

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

  return node;
}

int main() {
    Eigen::Vector2f in_1(2.0f, 4.0f);
    Eigen::Vector2f in_2(1.0f, 6.0f);
    Eigen::Vector2f in_3(9.0f, 2.0f);
    Eigen::Vector2f in_4(1.0f, 3.5f);
    Eigen::Vector2f in_5(3.0f, 8.0f);

    std::vector<Eigen::Vector2f> input_vec;
    input_vec.push_back(in_1);
    input_vec.push_back(in_2);
    input_vec.push_back(in_3);
    input_vec.push_back(in_4);
    input_vec.push_back(in_5);
    
    KDTree tree;
    auto node = tree.buildTree(input_vec);

    std::cout <<"Done";
    std::cout <<"Done";
}

