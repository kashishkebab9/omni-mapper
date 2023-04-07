#include "icp.h"

void icp::pclCallback(const sensor_msgs::PointCloud::ConstPtr&  msg)
{
  std::cout << "Received Pointcloud!" << std::endl;
  this->msg_t_minus_1 = this->msg_t;
  this->msg_t.clear();

  for (const auto& point : msg->points) {
    Eigen::Vector3f point_eigen(point.x, point.y, 1);
    this->msg_t.push_back(point_eigen);
  }

  if (!this->enough_msgs) {

    if(!msg_t_minus_1.size() == 0) {
      this->enough_msgs = true;
    }
  }

  if(this->enough_msgs) {
    // lets publish markers to make sure this polar->cartesian transformation was successful
   // visualization_msgs::Marker cartesian_points;
   // cartesian_points.action = visualization_msgs::Marker::DELETEALL;
   // cartesian_points.header.frame_id = "scarab41/laser";
   // cartesian_points.ns = "cartesian";
   // cartesian_points.action = visualization_msgs::Marker::ADD;
   // cartesian_points.pose.orientation.w = 1.0;
   // cartesian_points.id = 0;
   // cartesian_points.type = visualization_msgs::Marker::POINTS;

   // cartesian_points.scale.x = 0.05;
   // cartesian_points.scale.y = 0.05;
   // cartesian_points.color.r = 1.0f;
   // cartesian_points.color.a = 1.0;

   // for (auto x: this->msg_t)  {
   //   geometry_msgs::Point point;
   //   point.x = x.x();
   //   point.y = x.y();
   //   point.z = 0;
   //   cartesian_points.points.push_back(point);
   // }

   //this->cartesian_points_pub.publish(cartesian_points);
    KDTree tree;
    tree.buildTree(this->msg_t);
    float error = 0;

    // copy the prev_msg, as we plan on modifying the contents within the transformation loop:
    std::vector<Eigen::Vector3f> prev_msg_copy(this->msg_t_minus_1);
    for (size_t i = 0; i < prev_msg_copy.size(); i++) {
      std::pair<KDNode*, float> neighbor = tree.icp_nearest_neighbor(prev_msg_copy[i]);
      error += neighbor.second;
    } 
    
    std::cout << "Initial Error: " << error << std::endl;

    float error_threshold = 10;
    bool error_is_decreasing = true;

    float error_copy = error + 1.0;
    Eigen::Matrix3f final_transform;
    final_transform.setIdentity();
    //lets visualize the prev_msg:
    visualization_msgs::Marker prev_scan;
    prev_scan.action = visualization_msgs::Marker::DELETEALL;
    prev_scan.header.frame_id = "scarab41/laser";
    prev_scan.ns = "prev_msg_scan";
    prev_scan.action = visualization_msgs::Marker::ADD;
    prev_scan.pose.orientation.w = 1.0;
    prev_scan.id = 0;
    prev_scan.type = visualization_msgs::Marker::POINTS;

    prev_scan.scale.x = 0.05;
    prev_scan.scale.y = 0.05;
    prev_scan.color.g = 1.0f;
    prev_scan.color.b = 1.0f;
    prev_scan.color.a = 1.0;

    for (auto x: prev_msg_copy)  {
      geometry_msgs::Point point;
      point.x = x.x();
      point.y = x.y();
      point.z = 0;
      prev_scan.points.push_back(point);
    }

    this->prev_scan_pub.publish(prev_scan);

    while (error > error_threshold && error < error_copy) {
      std::cout << "Entered while loop" << std::endl;
      std::cout << "error: " << error <<  std::endl;
      std::cout << "error_copy: " << error_copy <<  std::endl;
      error_copy = error;

      Eigen::Matrix3f transformation = this->solve_transformation_loop(prev_msg_copy, this->msg_t);
      std::cout << "Transform: " << transformation.matrix() << std::endl;
      std::vector<Eigen::Vector3f> transformed_prev_msg = this->apply_transformation(prev_msg_copy,  transformation);

      float tracking_error=0;

      for (size_t i = 0; i < transformed_prev_msg.size(); i++) {
        std::pair<KDNode*, float> loop_error = tree.icp_nearest_neighbor(transformed_prev_msg[i]);
        if (loop_error.second < 20) {
          tracking_error += loop_error.second;
        }
      } 
      std::cout << "tracking_error: " << tracking_error << std::endl;

      if(error > tracking_error) {
        std::cout << "error is greater than tracking error!" << std::endl;
        final_transform = transformation * final_transform; 
        error = tracking_error;
        prev_msg_copy.clear();
        prev_msg_copy = transformed_prev_msg;
      } else {
        std::cout << "error is less than tracking error!" << std::endl;
        error_is_decreasing = false;
      }
    }
    std::cout << "Final Transformation: " << std::endl << final_transform.matrix() << std::endl;
    //lets visualize this transformed msg:
    std::vector<Eigen::Vector3f> transformed_prev_msg = this->apply_transformation(this->msg_t_minus_1,  final_transform);

    visualization_msgs::Marker transformed_scan;
    transformed_scan.action = visualization_msgs::Marker::DELETEALL;
    transformed_scan.header.frame_id = "scarab41/laser";
    transformed_scan.ns = "centroids";
    transformed_scan.action = visualization_msgs::Marker::ADD;
    transformed_scan.pose.orientation.w = 1.0;
    transformed_scan.id = 0;
    transformed_scan.type = visualization_msgs::Marker::POINTS;

    transformed_scan.scale.x = 0.05;
    transformed_scan.scale.y = 0.05;
    transformed_scan.color.g = 1.0f;
    transformed_scan.color.a = 1.0;

    for (auto x: transformed_prev_msg)  {
      geometry_msgs::Point point;
      point.x = x.x();
      point.y = x.y();
      point.z = 0;
      transformed_scan.points.push_back(point);
    }

    this->transformed_scan_pub.publish(transformed_scan);


  }
}

Eigen::Vector2f icp::calculateCentroid(std::vector<Eigen::Vector3f> input_set) {
  float avg_x = 0;
  float avg_y = 0;

  for (auto i: input_set) {
    if (i.x() != 0 && i.y() != 0 && i.x() !=-0 && i.y() != -0 ) {
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
  
  //lets put some markers down for the centroid:
  visualization_msgs::Marker centroid_marker;
  centroid_marker.action = visualization_msgs::Marker::DELETEALL;
  centroid_marker.header.frame_id = "scarab41/laser";
  centroid_marker.ns = "centroids";
  centroid_marker.action = visualization_msgs::Marker::ADD;
  centroid_marker.pose.orientation.w = 1.0;
  centroid_marker.id = 0;
  centroid_marker.type = visualization_msgs::Marker::POINTS;

  centroid_marker.scale.x = 0.2;
  centroid_marker.scale.y = 0.2;
  centroid_marker.color.b = 1.0f;
  centroid_marker.color.a = 1.0;

  geometry_msgs::Point prev_marker, current_marker;
  prev_marker.x = prev_centroid.x();
  prev_marker.y = prev_centroid.y();
  prev_marker.z = 0; 
  current_marker.x = current_centroid.x();
  current_marker.y = current_centroid.y();
  current_marker.z = 0; 

  centroid_marker.points.push_back(prev_marker);
  centroid_marker.points.push_back(current_marker);
  this->scan_centroid_pub.publish(centroid_marker);

  //we need to generate variance prime vec:
  // TODO: I believe this is the problem with my code. These matrices are not the shape we thought
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
    Eigen::Matrix2f w_loop = prev_prime[i] * current_prime[i].transpose();
    W_final += w_loop;
  }

  std::cout << "W_final: " << W_final.matrix() << std::endl;
  W_final = W_final.array()/summation_iteration;
  std::cout << "W_final: " << W_final.matrix() << std::endl;

  //perform mentioned SVD:
  Eigen::JacobiSVD<Eigen::Matrix2f> svd(W_final, Eigen::ComputeFullU | Eigen::ComputeFullV);
  
  //get our transformation Matrix:
  Eigen::Matrix2f rotation = svd.matrixU() * svd.matrixV().transpose();
  Eigen::Vector2f translation = current_centroid - (rotation * prev_centroid);
  Eigen::Matrix3f transformation;
  transformation.setIdentity();
  transformation.block<2,2>(0,0) = rotation;
  transformation.block<2,1>(0,2) = translation;
  std::cout << "Rotation: " << std::endl << rotation.matrix() << std::endl;
  std::cout << "Translation: " << std::endl << translation.matrix() << std::endl;
  std::cout << "Transformation: " << std::endl << transformation.matrix() << std::endl;
  return transformation;
}
//********************************************************************************//
std::vector<Eigen::Vector2f> icp::make_prime_vec(std::vector<Eigen::Vector3f> msg, Eigen::Vector2f input_centroid) {
  //prime vec is simply for each point in a message, subtract the centroid from it and return the modified message
  std::vector<Eigen::Vector2f> prime_vec;
  ROS_DEBUG("Making Prime Vec");
  for(size_t i = 0; i < msg.size(); i++) {
    float x = (msg[i]).x() - input_centroid.x();
    float y = (msg[i]).y() - input_centroid.y();

    Eigen::Vector2f prime_element;
    prime_element << x, y;
    prime_vec.push_back(prime_element);
  } 
  return prime_vec;
}

std::vector<Eigen::Vector3f> icp::apply_transformation(std::vector<Eigen::Vector3f> msg, Eigen::Matrix3f transformation) {
  std::vector<Eigen::Vector3f> output;

  for (size_t i = 0; i < msg.size(); i++) {
    Eigen::Vector3f transformed_vector = transformation * msg[i];
    output.push_back(transformed_vector);
  }
  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp_node");
  icp solver;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}

