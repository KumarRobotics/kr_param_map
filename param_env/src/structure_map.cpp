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
#include <std_msgs/Bool.h>

#include <Eigen/Eigen>
#include <random>

#include <map_utils/map_basics.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>
#include <map_utils/struct_map_gen.hpp>


using namespace std;

std::string _frame_id;
ros::Publisher _all_map_cloud_pub;
ros::Subscriber _res_sub, _gen_map_sub;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

param_env::StructMapGenerator _struct_map_gen;
param_env::GridMapParams _grid_mpa;
param_env::MapGenParams _map_gen_pa;

int _samples_on_map = 100;
int _num = 0.0, _initial_num;
bool _save_map = false, _auto_gen = false;
std::string _dataset_path;
double _seed; //random seed


void pubSensedPoints()
{
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);

  if(_save_map)
  {
    pcl::io::savePCDFileASCII(_dataset_path + std::string("pt") + std::to_string(_initial_num + _num) + std::string(".pcd"), cloudMap);
  }

  return;
}

void resCallback(const std_msgs::Float32 &msg)
{

  _grid_mpa.resolution_ = msg.data;

  _struct_map_gen.initParams(_grid_mpa);
  _struct_map_gen.resetMap();
  _struct_map_gen.getPC(cloudMap);

  pubSensedPoints();
}

void genMapCallback(const std_msgs::Bool &msg)
{  
  _seed += 1.0;
  _struct_map_gen.clear();
  _struct_map_gen.change_ratios(_seed);
  _struct_map_gen.getPC(cloudMap);
  _num += 1;

  pubSensedPoints();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "structure_map");
  ros::NodeHandle nh("~");

  _all_map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);

  _res_sub     = nh.subscribe("change_res", 10, resCallback);
  _gen_map_sub = nh.subscribe("change_map", 10, genMapCallback);

  param_env::BasicMapParams _mpa;

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);
  nh.param("map/resolution", _grid_mpa.resolution_, 0.1);

  _grid_mpa.basic_mp_ = _mpa;

  nh.param("map/frame_id", _frame_id, string("map"));
  nh.param("map/auto_change", _auto_gen, false);

  // parameters for the environment
  nh.param("params/cylinder_ratio", _map_gen_pa.cylinder_ratio_, 0.1);
  nh.param("params/circle_ratio", _map_gen_pa.circle_ratio_, 0.1);
  nh.param("params/gate_ratio", _map_gen_pa.gate_ratio_, 0.1);
  nh.param("params/ellip_ratio", _map_gen_pa.ellip_ratio_, 0.1);
  nh.param("params/poly_ratio", _map_gen_pa.poly_ratio_, 0.1);
  // random number ranges
  nh.param("params/w1", _map_gen_pa.w1_, 0.3);
  nh.param("params/w2", _map_gen_pa.w2_, 1.0);
  nh.param("params/w3", _map_gen_pa.w3_, 2.0);
  nh.param("params/w4", _map_gen_pa.w4_, 3.0);
  nh.param("params/add_noise", _map_gen_pa.add_noise_, false);
  nh.param("params/seed", _seed, 1.0);


  nh.param("dataset/save_map", _save_map, false);
  nh.param("dataset/samples_num", _samples_on_map, 0);
  nh.param("dataset/start_index", _initial_num, 0);
  nh.param("dataset/path", _dataset_path, std::string("path"));


  // origin mapsize resolution isrequired
  ros::Rate loop_rate(5.0);

  if(opendir(_dataset_path.c_str()) == NULL)
  {
    string cmd = "mkdir -p " + _dataset_path;
    system(cmd.c_str());
  }

  ros::Duration(0.5).sleep();

  
  _struct_map_gen.initParams(_grid_mpa);
  _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed);
  _struct_map_gen.getPC(cloudMap);
  _num += 1;
  pubSensedPoints();
  loop_rate.sleep();

  
  while (ros::ok())
  {
    if (_auto_gen && _num < _samples_on_map)
    {
      _seed += 1.0;
      _struct_map_gen.clear();
      _struct_map_gen.change_ratios(_seed);
      _struct_map_gen.getPC(cloudMap);
      _num += 1;
      pubSensedPoints();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
