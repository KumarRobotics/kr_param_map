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
#include <std_msgs/Float32.h>
#include <Eigen/Eigen>
#include <random>

#include <map_utils/closed_shapes.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>

using namespace std;

std::string _frame_id;
ros::Publisher  _all_map_cloud_pub;
ros::Subscriber _res_sub;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

double _resolution;

param_env::StructMapGenerator _struct_map_gen;

void resCallback(const std_msgs::Float32 &msg){

  _struct_map_gen.resetRes(msg.data);
  _struct_map_gen.getPoints(cloudMap);

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
  _res_sub = nh.subscribe("change_res", 10, resCallback);

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

  double cylinder_ratio, circle_ratio, gate_ratio, ellip_ratio, poly_ratio;
  // radio of obstacles
  nh.param("map/cylinder_ratio", cylinder_ratio, 0.1);
  nh.param("map/circle_ratio", circle_ratio, 0.1);
  nh.param("map/gate_ratio", gate_ratio, 0.1);
  nh.param("map/ellip_ratio", ellip_ratio, 0.1);
  nh.param("map/poly_ratio", poly_ratio, 0.1);

  _cylinder_grids = ceil(_all_grids * cylinder_ratio);
  _circle_grids = ceil(_all_grids * circle_ratio);
  _gate_grids = ceil(_all_grids * gate_ratio);
  _ellip_grids = ceil(_all_grids * ellip_ratio);
  _poly_grids = ceil(_all_grids * poly_ratio);

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