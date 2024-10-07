#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "ros/ros.h"
#include <laser_geometry/laser_geometry.h>

#include "std_msgs/String.h"
class converter {
  public:
    converter() {
      scan_sub = nh.subscribe("scarab41/scan", 1, &converter::scanCallback, this);
      pointcloud_pub = nh.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  private:
    ros::NodeHandle nh, pnh;
    ros::Subscriber scan_sub;

    ros::Publisher pointcloud_pub;

    laser_geometry::LaserProjection projector_;

};

void converter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  sensor_msgs::PointCloud cloud;
  this->projector_.projectLaser(*msg, cloud);
  this->pointcloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_converter");
  converter convert_;

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
