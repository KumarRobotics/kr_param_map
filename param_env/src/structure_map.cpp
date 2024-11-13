#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <map_utils/geo_map.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/map_basics.hpp>
#include <map_utils/struct_map_gen.hpp>
#include <random>

using namespace std;

std::string _frame_id;
ros::Publisher _all_map_cloud_pub;
ros::Subscriber _res_sub, _gen_map_sub;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

param_env::StructMapGenerator _struct_map_gen;
param_env::GridMapParams _grid_mpa;
param_env::MapGenParams _map_gen_pa;
double _inflate_radius = 0.0;
int _samples_on_map = 100;
int _num = 0.0, _initial_num;
bool _save_map = false, _auto_gen = false;
std::string _dataset_path;
std::string _clear_path;
double _seed;  // random seed
bool clear_3d = true, _clear_pos = false;
std::vector<Eigen::Vector3d> clear_pos;

// safe 2d map 
bool _simple_2d = false;

// dyn part
double dt = 0.5;
bool dyn_mode = true;

void readClearPos()
{
  std::ifstream fp(_clear_path);
  std::string line;
  Eigen::Vector3d data_line;
  std::cout << "_clear_path: "<< _clear_path<<std::endl;
  while (getline(fp, line))
  { 
    std::string number;
    std::istringstream readstr(line);
    int i = 0;
    while (getline(readstr,number,','))
    {
      std::cout << "number: "<< number<<std::endl;
      data_line(i) = atof(number.c_str());
      i += 1;
    }
    clear_pos.push_back(data_line);
    if (i == 2)
    {
      clear_3d = false;
    }
  }
  return;
}

void clearCloud()
{
  std::cout << "start clear points, the cloudMap.size(): "<< cloudMap.size()<<std::endl;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloudMap.begin();
            it != cloudMap.end();)
    {
       Eigen::Vector3d pt(it->x, it->y, it->z);
      bool remove = false;
        if (clear_3d)
        {
          for (int j = 0; j < clear_pos.size(); j++)
          {
            if ((pt - clear_pos[j]).norm() < 2.0)
            {
              remove = true;
              break;
            }
          }
        }else
        {
          for (int j = 0; j < clear_pos.size(); j++)
          {
            if ((pt.head(2) - clear_pos[j].head(2)).norm() < 2.0)
            {
              remove = true;
              //it = cloudMap.erase(it);   // erase twice, will cause it out of range
              break;
            }
          }
        }

        if(remove)
            it = cloudMap.erase(it);
        else
            ++it;
    }

  std::cout << "cloudMap.size(): "<< cloudMap.size()<<std::endl;
  return;
}



void pubSensedPoints() {
  
  if (_simple_2d)
  {
    _struct_map_gen.getPC2D(cloudMap);
  }
  else
  {
    _struct_map_gen.getPC(cloudMap);
  }

  // filter map
  if (_clear_pos)
  {
    clearCloud();
  }
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  _all_map_cloud_pub.publish(globalMap_pcd);
  if (_save_map) {
    pcl::io::savePCDFileASCII(_dataset_path + std::string("pt") +
                                  std::to_string(_initial_num + _num) +
                                  std::string(".pcd"),
                              cloudMap);

  }
  return;
}




void resCallback(const std_msgs::Float32& msg) {

  float res = msg.data;
  float inv_res = 1.0 / res;

  if (inv_res - float((int)inv_res) < 1e-6) 
  {
    _grid_mpa.resolution_ = res;

    _struct_map_gen.changeRes(_grid_mpa.resolution_);
    _struct_map_gen.resetMap();

    pubSensedPoints();

  }
  else
  {
    ROS_WARN("The resolution is not valid! Try a different one !");
  }


}

void genMapCallback(const std_msgs::Bool& msg) {
  _seed += 1.0;
  _struct_map_gen.clear();
  _struct_map_gen.change_ratios(_seed, false, dt);
  _num += 1;

  pubSensedPoints();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "param_map");
  ros::NodeHandle nh("~");

  _all_map_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);

  _res_sub = nh.subscribe("change_res", 10, resCallback);
  _gen_map_sub = nh.subscribe("change_map", 10, genMapCallback);

  param_env::BasicMapParams _mpa;

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);
  nh.param("map/resolution", _grid_mpa.resolution_, 0.1);
  nh.param("map/inflate_radius", _inflate_radius, 0.1);
  nh.param("map/simple_2d", _simple_2d, false);

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
  nh.param("params/add_noise", _map_gen_pa.add_noise_, false);
  nh.param("params/seed", _seed, 1.0);

  nh.param("dataset/save_map", _save_map, false);
  nh.param("dataset/samples_num", _samples_on_map, 0);
  nh.param("dataset/start_index", _initial_num, 0);
  nh.param("dataset/path", _dataset_path, std::string("path"));

  nh.param("clear_path", _clear_path, string("pos.csv"));
  nh.param("clear_pos", _clear_pos, false);


  // dynamic movement with constant velocity 
  nh.param("dyn/v_x_h", _struct_map_gen.vel_h(0), 0.1);
  nh.param("dyn/v_y_h", _struct_map_gen.vel_h(1), 0.1);
  nh.param("dyn/v_z_h", _struct_map_gen.vel_h(2), 0.0);
  nh.param("dyn/dt", dt, 10.0);
  nh.param("dyn/dyn_mode", dyn_mode, false);

  if(!dyn_mode) readClearPos();

  // origin mapsize resolution isrequired
  ros::Rate loop_rate(dt);

  if (opendir(_dataset_path.c_str()) == NULL) {
    string cmd = "mkdir -p " + _dataset_path;
    system(cmd.c_str());
  }

  ros::Duration(1.0).sleep();
  _struct_map_gen.initParams(_grid_mpa);
  _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, dyn_mode);  // generate the initial obs
  _num += 1;
  pubSensedPoints();
  loop_rate.sleep();
  param_env::StructMapGenerator temp;

  while (ros::ok()) {
    // random map gen
    if(!dyn_mode){
      if (_auto_gen && _num < _samples_on_map) {
        _seed += 1.0;
        _struct_map_gen.clear();
        _struct_map_gen.change_ratios(_seed, false, dt);
    
        _num += 1;
        pubSensedPoints();
      }
    }
    else{ //dynamic obs:
      _struct_map_gen.clear();
      _struct_map_gen.dyn_generate(1/dt);
      pubSensedPoints();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}