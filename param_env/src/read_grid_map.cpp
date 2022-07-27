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

#include <map_utils/map_basics.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>
#include <map_utils/struct_map_gen.hpp>


using namespace std;

std::string _frame_id;
ros::Publisher  _all_map_cloud_pub;
ros::Subscriber _res_sub;

sensor_msgs::PointCloud2 globalMap_pcd;

/*** global params for cloudMap***/
pcl::PointCloud<pcl::PointXYZ> cloudMap;

param_env::StructMapGenerator _struct_map_gen;
param_env::MapParams _mpa;
param_env::MapGenParams _mgpa;


/*** read ros bag ***/
void read_pcs_bag(std::string file_name, std::string topic, sensor_msgs::PointCloud &cloud) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool find = false;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (m.instantiate<T>() != NULL) {
      msg = *m.instantiate<T>();
      ROS_WARN("Get data!");
      find = true;
      break;
    }
  }
  bag.close();
  if (!find)
    ROS_WARN("Fail to find '%s' in '%s'", topic.c_str(), file_name.c_str());

  
  //should be 2
  sensor_msgs::PointCloud2 cloud2 = read_bag<sensor_msgs::PointCloud2>(file_name, topic_name);
  //Convert into vector of Eigen
   cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud);
  cloud.header = header_;
  map_pub.publish(cloud);

  return msg;
}

/*** read pcd file ***/


/*** randomly gen points  ***/
void gen_pcs(float bound=50, int num = 10000){
  
  std::default_random_engine random(time(NULL));
  std::uniform_real_distribution<double> r(-bound, bound);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int loop_n = 0;

  while (loop_n < num) {
      float ax = r(random);
      float ay = r(random);
      float az = r(random);
      float fx1 = bound/4.0;
      float fy1 = bound/4.0;
      float fy2 = bound/4.0;
      float fz2 = bound/4.0;
      if (ax < fx1 && ax > - fx1 && ay < fy1 && ay > - fy1) continue;
      if (ay < fy2 && ay > - fy2 && az < fz2 && az > - fz2) continue;
      cloud->points.push_back(pcl::PointXYZ(ax,ay,az));
      loop_n++;
  }

}



void resCallback(const std_msgs::Float32 &msg){

  _mpa.resolution_ = msg.data;

  _struct_map_gen.initParams(_mpa);
  _struct_map_gen.resetMap();

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

  _all_map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_gridmap", 1);
  _res_sub = nh.subscribe("change_res", 10, resCallback);

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);
  nh.param("map/resolution", _mpa.resolution_, 0.1);

  nh.param("map/frame_id", _frame_id, string("map"));

  // parameters for the environment
  nh.param("map/cylinder_ratio", _mgpa.cylinder_ratio_, 0.1);
  nh.param("map/circle_ratio", _mgpa.circle_ratio_, 0.1);
  nh.param("map/gate_ratio", _mgpa.gate_ratio_, 0.1);
  nh.param("map/ellip_ratio", _mgpa.ellip_ratio_, 0.1);
  nh.param("map/poly_ratio", _mgpa.poly_ratio_, 0.1);
  // random number ranges
  nh.param("params/w1", _mgpa.w1_, 0.3);
  nh.param("params/w2", _mgpa.w2_, 1.0);
  nh.param("params/w3", _mgpa.w3_, 2.0);
  nh.param("params/w4", _mgpa.w4_, 3.0);

  _struct_map_gen.initParams(_mpa);
  _struct_map_gen.randomUniMapGen(_mgpa);
  
  _struct_map_gen.getPC(cloudMap);
  ros::Duration(0.5).sleep();

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}