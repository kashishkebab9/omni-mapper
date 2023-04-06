#include <iterator>
#include <math.h>
#include <boost/bind/bind.hpp>
#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>

#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
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
      pcl_sub = nh.subscribe("pointcloud", 1, &icp::pclCallback, this);
      scan_centroid_pub = nh.advertise<visualization_msgs::Marker>("scan_centroid", 1);
      transformed_scan_pub = nh.advertise<visualization_msgs::Marker>("transformed_scan", 1);
      prev_scan_pub = nh.advertise<visualization_msgs::Marker>("prev_scan", 1);
      cartesian_points_pub = nh.advertise<visualization_msgs::Marker>("cartesian_points", 1);
    }

    void pclCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    Eigen::Matrix3f solve_transformation_loop(std::vector<Eigen::Vector3f> prev_scan, std::vector<Eigen::Vector3f> current_scan);
    Eigen::Vector2f calculateCentroid(std::vector<Eigen::Vector3f> input_set);
    std::vector<Eigen::Vector2f> make_prime_vec(std::vector<Eigen::Vector3f> msg, Eigen::Vector2f avg);
    std::vector<Eigen::Vector3f> apply_transformation(std::vector<Eigen::Vector3f> msg, Eigen::Matrix3f transformation);
    //laser_geometry::LaserProjection projector_;

  
  private:
    ros::NodeHandle nh, pnh;
    ros::Subscriber pcl_sub;

    ros::Publisher cartesian_points_pub;
    ros::Publisher scan_centroid_pub;
    ros::Publisher transformed_scan_pub;
    ros::Publisher prev_scan_pub;

    std::vector<Eigen::Vector3f> msg_t_minus_1;
    std::vector<Eigen::Vector3f> msg_t;
    bool enough_msgs;

};

