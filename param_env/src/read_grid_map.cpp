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
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <Eigen/Eigen>
#include <random>

#include <map_utils/map_basics.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

std::string _frame_id;
ros::Publisher _all_cloud_pub, _all_map_pub;
ros::Subscriber _res_sub;

sensor_msgs::PointCloud2 globalCloud_pcd, globalMap_pcd;

/*** global params for cloudMap***/
pcl::PointCloud<pcl::PointXYZ> cloudMap, gridCloudMap;

param_env::GridMapParams _grid_mpa;
param_env::GridMap _grid_map;
param_env::BasicMapParams _mpa;
double _inflate_ratio = 0.0;


void toPcsMsg()
{
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalCloud_pcd);
}






/*** read ros bag ***/
template <class T>
void read_pcs_bag(std::string &path, std::string &topic, T &msg)
{
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool find = false;
  BOOST_FOREACH (rosbag::MessageInstance const m, view)
  {

    if (m.instantiate<T>() != NULL)
    {
      msg = *m.instantiate<T>();
      ROS_WARN("Get data!");
      find = true;
      break;
    }
  }
  bag.close();
  if (!find)
    ROS_WARN("Fail to find '%s' in '%s'", topic.c_str(), path.c_str());

  return;
}

/*** read pcd file ***/
void read_pcs_pcd(std::string &path)
{
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, cloudMap) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
  }

  toPcsMsg();
}


/*** randomly gen points  ***/
void gen_pcs(float bound = 50, int num = 10000)
{

  std::default_random_engine random(time(NULL));
  std::uniform_real_distribution<double> r(-bound, bound);
  int loop_n = 0;

  while (loop_n < num)
  {
    float ax = r(random);
    float ay = r(random);
    float az = r(random);
    float fx1 = bound / 4.0;
    float fy1 = bound / 4.0;
    float fy2 = bound / 4.0;
    float fz2 = bound / 4.0;
    if (ax < fx1 && ax > -fx1 && ay < fy1 && ay > -fy1)
      continue;
    if (ay < fy2 && ay > -fy2 && az < fz2 && az > -fz2)
      continue;
    cloudMap.points.push_back(pcl::PointXYZ(ax, ay, az));
    loop_n++;
  }

  toPcsMsg();
}

void resCallback(const std_msgs::Float32 &msg)
{

  _grid_mpa.resolution_ = msg.data;
  _grid_map.clearAllOcc();
  _grid_map.fillMap(cloudMap, _inflate_ratio);
  _grid_map.publishMap(gridCloudMap);

}

void pubSensedPoints()
{

  globalCloud_pcd.header.frame_id = _frame_id;
  _all_cloud_pub.publish(globalCloud_pcd);

  pcl::toROSMsg(gridCloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_pub.publish(globalMap_pcd);

  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "structure_map");
  ros::NodeHandle nh("~");

  _all_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_gridmap", 1);

  _res_sub = nh.subscribe("change_res", 10, resCallback);

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);

  nh.param("map/resolution", _grid_mpa.resolution_, 0.1);
  nh.param("map/frame_id", _frame_id, string("map"));
  nh.param("map/inflate_ratio", _inflate_ratio, 0.1);


  //set up basic parameters for grid map
  _grid_mpa.basic_mp_ = _mpa;
  _grid_mpa.basic_mp_.min_range_ = _grid_mpa.basic_mp_.map_origin_;
  _grid_mpa.basic_mp_.max_range_ = _grid_mpa.basic_mp_.map_origin_ + _grid_mpa.basic_mp_.map_size_;
  _grid_mpa.basic_mp_.map_volume_ = _grid_mpa.basic_mp_.map_size_(0) * _grid_mpa.basic_mp_.map_size_(1) * _grid_mpa.basic_mp_.map_size_(2);

  _grid_map.initMap(_grid_mpa);


  // map mode
  // 0 --- randomly generate
  // 1 --- read the ros bag poind cloud 1
  // 2 --- read the ros bag poind cloud 2
  // 3 --- read pcd file
  int mode;
  nh.param("map/mode", mode, 0);

  std::string file_path, topic_name;

  nh.param("file_path", file_path, std::string("path"));
  nh.param("bag_topic", topic_name, std::string("point_clouds_topic"));


  switch (mode)
  {
  case 0:
    gen_pcs();
    break;
  case 1:
  {
    sensor_msgs::PointCloud msg;
    read_pcs_bag(file_path, topic_name, msg);
    convertPointCloudToPointCloud2(msg, globalCloud_pcd);
    pcl::fromROSMsg(globalCloud_pcd, cloudMap);
    break;
  }
  case 2:
  {
    read_pcs_bag(file_path, topic_name, globalCloud_pcd);
    pcl::fromROSMsg(globalCloud_pcd, cloudMap);
    break;
  }
  case 3:
    read_pcs_pcd(file_path);
    break;
  }

  _grid_map.fillMap(cloudMap, _inflate_ratio);
  _grid_map.publishMap(gridCloudMap);



  ros::Duration(0.5).sleep();
  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}