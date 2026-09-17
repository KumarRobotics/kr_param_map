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
ros::Publisher _all_moving_cloud_pub , _all_static_cloud_pub, _obs_info_pub;
ros::Subscriber _res_sub, _gen_map_sub;

sensor_msgs::PointCloud2 globalMap_pcd, globalMap_moving_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap, movingCloudMap;

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

param_env::BasicMapParams _mpa;

// dyn part
double dt = 0.5;
enum {
    BothDynStaticMode = 0, OnlyDynMode = 1, OnlyStaticMode = 2, RandomStaticMode = 3
};
int MapMode;
double horizon, last_send_t = 0.0;


void readClearPos()
{
  std::ifstream fp(_clear_path);
  std::string line;
  Eigen::Vector3d data_line;
  //std::cout << "_clear_path: "<< _clear_path<<std::endl;
  while (getline(fp, line))
  { 
    std::string number;
    std::istringstream readstr(line);
    int i = 0;
    while (getline(readstr,number,','))
    {
      //std::cout << "number: "<< number<<std::endl;
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
  //std::cout << "start clear points, the cloudMap.size(): "<< cloudMap.size()<<std::endl;
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
  return;
}


void pubSensedPoints(int mode) {
  switch (mode) {
      case OnlyDynMode:
          //std::cout << "OnlyDynMode: movingCloudMap.size(): "<< movingCloudMap.size()<<std::endl;
          _struct_map_gen.getMovingPC(movingCloudMap);
          pcl::toROSMsg(movingCloudMap, globalMap_moving_pcd);
          globalMap_moving_pcd.header.frame_id = _frame_id;
          _all_moving_cloud_pub.publish(globalMap_moving_pcd);
          break;
      case OnlyStaticMode:
          //std::cout << "OnlyStaticMode: cloudMap.size(): "<< cloudMap.size()<<std::endl;
          _struct_map_gen.getPC(cloudMap);
          pcl::toROSMsg(cloudMap, globalMap_pcd);
          globalMap_pcd.header.frame_id = _frame_id;
          _all_static_cloud_pub.publish(globalMap_pcd);
          break;
      case BothDynStaticMode:
          //std::cout << "BothDynStaticMode: movingCloudMap.size(): "<< movingCloudMap.size()<<std::endl;
          //std::cout << "BothDynStaticMode: cloudMap.size(): "<< cloudMap.size()<<std::endl;
          _struct_map_gen.getMovingPC(movingCloudMap);
          pcl::toROSMsg(movingCloudMap, globalMap_moving_pcd);
          globalMap_moving_pcd.header.frame_id = _frame_id;
          _all_moving_cloud_pub.publish(globalMap_moving_pcd);
          _struct_map_gen.getPC(cloudMap);
          pcl::toROSMsg(cloudMap, globalMap_pcd);
          globalMap_pcd.header.frame_id = _frame_id;
          _all_static_cloud_pub.publish(globalMap_pcd);
          break;
      case RandomStaticMode:
          //std::cout << "RandomStaticMode: cloudMap.size(): "<< cloudMap.size()<<std::endl;
          _struct_map_gen.getPC(cloudMap);
          // filter map
          if (_clear_pos)
          {
              clearCloud();
          }
          pcl::toROSMsg(cloudMap, globalMap_pcd);
          globalMap_pcd.header.frame_id = _frame_id;
          _all_static_cloud_pub.publish(globalMap_pcd);
          if (_save_map) {
              pcl::io::savePCDFileASCII(_dataset_path + std::string("pt") +
                                        std::to_string(_initial_num + _num) +
                                        std::string(".pcd"),
                                        cloudMap);
          }
          break;
  }
}


bool pubObsInfo(double time_now, double last_send_time){
  if(time_now - last_send_time < horizon){
      return false;
  }
  plan_env::Obsinfo info;
  _struct_map_gen.SendObsTrajInfo(info, horizon, time_now);  // send dynamic obs info to planner
  _obs_info_pub.publish(info);
  return true;
}

void resCallback(const std_msgs::Float32& msg) {

  float res = msg.data;
  float inv_res = 1.0 / res;

  if (inv_res - float((int)inv_res) < 1e-6) 
  {
    _grid_mpa.resolution_ = res;
    _struct_map_gen.changeRes(_grid_mpa.resolution_);
    _struct_map_gen.resetMap();
    pubSensedPoints(RandomStaticMode);

  }
  else
  {
    ROS_WARN("The resolution is not valid! Try a different one !");
  }


}

void genMapCallback(const std_msgs::Bool& msg) {
  _seed += 1.0;
  _struct_map_gen.clear();
  _struct_map_gen.change_ratios(_seed, dt);
  _num += 1;
  pubSensedPoints(RandomStaticMode);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "param_map");
  ros::NodeHandle nh("~");

  _all_static_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1); // to rviz and to /pcl/render(remap to /gridmap/cloud)

  _all_moving_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("moving_cloud", 1); // to rviz

  _obs_info_pub = nh.advertise<plan_env::Obsinfo>("obsinfo_topic",1); // to planner

  _res_sub = nh.subscribe("change_res", 10, resCallback);
  _gen_map_sub = nh.subscribe("change_map", 10, genMapCallback);

  //param_env::BasicMapParams _mpa;

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);
  nh.param("map/resolution", _grid_mpa.resolution_, 0.1);
  nh.param("map/inflate_radius", _inflate_radius, 0.1);

  _grid_mpa.basic_mp_ = _mpa;

  nh.param("map/frame_id", _frame_id, string("map"));
  nh.param("map/auto_change", _auto_gen, false);

  // parameters for the environment
  nh.param("params/static_cylinder_ratio", _map_gen_pa.static_cylinder_ratio_, 0.1);
  nh.param("params/static_circle_ratio", _map_gen_pa.static_circle_ratio_, 0.1);
  nh.param("params/static_gate_ratio", _map_gen_pa.static_gate_ratio_, 0.1);
  nh.param("params/static_ellip_ratio", _map_gen_pa.static_ellip_ratio_, 0.1);
  nh.param("params/static_poly_ratio", _map_gen_pa.static_poly_ratio_, 0.1);
  nh.param("params/static_ball_ratio_", _map_gen_pa.static_ball_ratio_, 0.1);
    // parameters for the environment
  nh.param("params/dyn_cylinder_ratio", _map_gen_pa.dyn_cylinder_ratio_, 0.1);
  nh.param("params/dyn_circle_ratio", _map_gen_pa.dyn_circle_ratio_, 0.1);
  nh.param("params/dyn_gate_ratio", _map_gen_pa.dyn_gate_ratio_, 0.1);
  nh.param("params/dyn_ellip_ratio", _map_gen_pa.dyn_ellip_ratio_, 0.1);
  nh.param("params/dyn_poly_ratio", _map_gen_pa.dyn_poly_ratio_, 0.1);
  nh.param("params/dyn_ball_ratio_", _map_gen_pa.dyn_ball_ratio_, 0.1);
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

  nh.param("dyn/v_x", _struct_map_gen.vel_h(0), 0.1);
  nh.param("dyn/v_y", _struct_map_gen.vel_h(1), 0.1);
  nh.param("dyn/v_z", _struct_map_gen.vel_h(2), 0.0);
  nh.param("dyn/v_x_max", _struct_map_gen.vel_bound(0), 0.1);
  nh.param("dyn/v_y_max", _struct_map_gen.vel_bound(1), 0.1);
  nh.param("dyn/v_z_max", _struct_map_gen.vel_bound(2), 0.0);
  nh.param("dyn/a_x_max", _struct_map_gen.acc_bound(0), 0.1);
  nh.param("dyn/a_y_max", _struct_map_gen.acc_bound(1), 0.1);
  nh.param("dyn/a_z_max", _struct_map_gen.acc_bound(2), 0.0);
  nh.param("dyn/dt", dt, 10.0);
  nh.param("dyn/map_mode", MapMode, 0);
  nh.param("dyn/horizon", horizon, 0.0);

  ros::Rate loop_rate(dt);
  ros::Duration(1.0).sleep();
  _struct_map_gen.initParams(_grid_mpa);

  // generate the initial obs
  if(MapMode == OnlyDynMode) {
      _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, true);
  }
  else if(MapMode == OnlyStaticMode){
      _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, false);
  }
  else if(MapMode == BothDynStaticMode){
      _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, true);
      _seed++;
      _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, false);
  }
  else if(MapMode == RandomStaticMode){
      readClearPos();
      if (opendir(_dataset_path.c_str()) == NULL) {
          string cmd = "mkdir -p " + _dataset_path;
          system(cmd.c_str());
      }
      _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, false);
      _num += 1;
  }
  else{
      // nothing
  }

  if(MapMode == OnlyDynMode || MapMode == BothDynStaticMode) {
      _struct_map_gen.GetObsPolyTraj();
      if(pubObsInfo(ros::Time::now().toSec(), 0.0)) {
          last_send_t = ros::Time::now().toSec();
      }

  }

  pubSensedPoints(MapMode);
  loop_rate.sleep();
  param_env::StructMapGenerator temp;

  while (ros::ok()) {
    // random map gen
    switch (MapMode) {
        case RandomStaticMode:
            if (_auto_gen && _num < _samples_on_map) {
                _seed += 1.0;
                _struct_map_gen.clear();
                _struct_map_gen.change_ratios(_seed, dt);
                _num += 1;
                pubSensedPoints(RandomStaticMode);
            }
            break;
        case OnlyDynMode:
            _struct_map_gen.dyn_generate_traj(ros::Time::now().toSec());
            if(pubObsInfo(ros::Time::now().toSec(), last_send_t)) { // to planner
                last_send_t = ros::Time::now().toSec();
            }
            pubSensedPoints(OnlyDynMode); // only rviz
            break;
        case OnlyStaticMode:
            pubSensedPoints(OnlyStaticMode); // only rviz
            break;
        case BothDynStaticMode:
            _struct_map_gen.dyn_generate_traj(ros::Time::now().toSec());
            if(pubObsInfo(ros::Time::now().toSec(), last_send_t)) { // to planner
                last_send_t = ros::Time::now().toSec();
            }
            pubSensedPoints(OnlyDynMode); // only rviz
            pubSensedPoints(OnlyStaticMode); // only rviz
            break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
