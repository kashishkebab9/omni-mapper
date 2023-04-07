#include "KDTree.h"

int main() {
  KDTree tree;

  // create a vector of points
  std::vector<Eigen::Vector3f> vec_to_test;
  float x, y; 

  for (int i =0; i< 100; i++) {
    Eigen::Vector3f point;
    x = float(rand()%100);
    y = float(rand()%100);

    point << x, y, 1;
    vec_to_test.push_back(point);
  }

  tree.buildTree(vec_to_test);
  Eigen::Vector3f input;
  input << 100, 0, 1;
  std::cout << "input: " << input.x() << ", " << input.y() << std::endl;

  tree.nearestNeighbor(input);


  

}
