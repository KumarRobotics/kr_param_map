#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <iomanip>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <random>

#include <map_utils/grid_map.hpp>

using namespace std;


std::string _frame_id;
ros::Publisher _all_map_cloud_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

random_device rd;
default_random_engine eng(rd());

uniform_real_distribution<double> rand_theta;

double _resolution;

double _w1, _w2, _w3, _w4;

// for normal cylinders
uniform_real_distribution<double> rand_w, rand_h, rand_cw, rand_radiu;

param_env::GridMap _grid_map;
param_env::MapParams mpa;


void RandomUniMapGenerate()
{
  Eigen::Vector3d bound;
  Eigen::Vector3d cpt; // center points, object points
  
  _grid_map.setUniRand(eng);



  pt_random.x = cpt(0);
  pt_random.y = cpt(1);
  pt_random.z = cpt(2);
  cloudMap.points.push_back(pt_random);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  std::cout << "Finished generate random map !" << std::endl;

}

int i = 0;
void pubSensedPoints()
{
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);
  // }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "structure_map");
  ros::NodeHandle nh("~");

  _all_map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);

  nh.param("map/x_size", mpa.map_size_(0), 40.0);
  nh.param("map/y_size", mpa.map_size_(1), 40.0);
  nh.param("map/z_size", mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", mpa.map_origin_(2), 0.0);
  nh.param("map/resolution", mpa.resolution_, 0.1);

  nh.param("map/frame_id", _frame_id, string("map"));

  // space volume
  _resolution = mpa.resolution_;
  _all_grids = mpa.map_size_(0) * mpa.map_size_(1) * mpa.map_size_(2) / std::pow(mpa.resolution_, 3);

  // low and high bound of the center position
  _grid_map.init(mpa);

  // parameters
  nh.param("params/w1", _w1, 0.3);
  nh.param("params/w2", _w2, 1.0);
  nh.param("params/w3", _w3, 2.0);
  nh.param("params/w4", _w4, 3.0);

  ros::Duration(0.5).sleep();

  RandomUniMapGenerate();

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}