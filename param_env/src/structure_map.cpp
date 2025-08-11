#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <math.h>
#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <random>

#include <map_utils/geo_map.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/map_basics.hpp>
#include <map_utils/struct_map_gen.hpp>



using namespace std;

std::string _frame_id;

using std::placeholders::_1;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_cloud_pub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _res_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _gen_map_sub;


sensor_msgs::msg::PointCloud2  globalMap_pcd;
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
    std::cout << "start clear points, the cloudMap.size(): " << cloudMap.size() << std::endl;

    const double radius_sq = 4.0; // 2.0^2
    const bool use3D = clear_3d;

    cloudMap.erase(
        std::remove_if(cloudMap.begin(), cloudMap.end(),
            [&](const pcl::PointXYZ &p) {
                Eigen::Vector3d pt(p.x, p.y, p.z);
                for (const auto &cp : clear_pos)
                {
                    double dist_sq = use3D ?
                        (pt - cp).squaredNorm() :
                        (pt.head<2>() - cp.head<2>()).squaredNorm();

                    if (dist_sq < radius_sq)
                        return true; // mark for removal
                }
                return false;
            }),
        cloudMap.end()
    );

    std::cout << "cloudMap.size(): " << cloudMap.size() << std::endl;
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
  // Convert PCL point cloud to ROS 2 PointCloud2 message
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  // Publish the point cloud
  _all_map_cloud_pub->publish(globalMap_pcd);

  if (_save_map) {
    pcl::io::savePCDFileASCII(_dataset_path + std::string("pt") +
                                  std::to_string(_initial_num + _num) +
                                  std::string(".pcd"),
                              cloudMap);

  }
  return;
}


// Change callbacks to accept shared_ptr<const MsgType> instead of MsgType&
void resCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  float res = msg->data;
  float inv_res = 1.0 / res;

  if (inv_res - float((int)inv_res) < 1e-6) {
    _grid_mpa.resolution_ = res;
    _struct_map_gen.changeRes(_grid_mpa.resolution_);
    _struct_map_gen.resetMap();
    pubSensedPoints();
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The resolution is not valid! Try a different one!");
  }
}

void genMapCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  _seed += 1.0;
  _struct_map_gen.clear();
  _struct_map_gen.change_ratios(_seed, false, dt);
  _num += 1;

  pubSensedPoints();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("param_map");

  _all_map_cloud_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_cloud", 1);

  _res_sub = nh->create_subscription<std_msgs::msg::Float32>(
      "change_res", 10, std::bind(&resCallback, _1));

  _gen_map_sub = nh->create_subscription<std_msgs::msg::Bool>(
      "change_map", 10, std::bind(&genMapCallback, _1));

  param_env::BasicMapParams _mpa;

  nh->declare_parameter<double>("map/x_size", 40.0);
  nh->declare_parameter<double>("map/y_size", 40.0);
  nh->declare_parameter<double>("map/z_size", 5.0);
  nh->declare_parameter<double>("map/x_origin", -20.0);
  nh->declare_parameter<double>("map/y_origin", -20.0);
  nh->declare_parameter<double>("map/z_origin", 0.0);
  nh->declare_parameter<double>("map/resolution", 0.1);
  nh->declare_parameter<double>("map/inflate_radius", 0.1);
  nh->declare_parameter<bool>("map/simple_2d", false);

  nh->get_parameter("map/x_size", _mpa.map_size_(0));
  nh->get_parameter("map/y_size", _mpa.map_size_(1));
  nh->get_parameter("map/z_size", _mpa.map_size_(2));
  nh->get_parameter("map/x_origin", _mpa.map_origin_(0));
  nh->get_parameter("map/y_origin", _mpa.map_origin_(1));
  nh->get_parameter("map/z_origin", _mpa.map_origin_(2));
  nh->get_parameter("map/resolution", _grid_mpa.resolution_);
  nh->get_parameter("map/inflate_radius", _inflate_radius);
  nh->get_parameter("map/simple_2d", _simple_2d);

  _grid_mpa.basic_mp_ = _mpa;

  nh->declare_parameter<std::string>("map/frame_id", "map");
  nh->get_parameter("map/frame_id", _frame_id);
  nh->declare_parameter<bool>("map/auto_change", false);
  nh->get_parameter("map/auto_change", _auto_gen);

  // declare and get other params similarly:
  nh->declare_parameter<double>("params/cylinder_ratio", 0.1);
  nh->get_parameter("params/cylinder_ratio", _map_gen_pa.cylinder_ratio_);
  nh->declare_parameter<double>("params/circle_ratio", 0.1);
  nh->get_parameter("params/circle_ratio", _map_gen_pa.circle_ratio_);
  nh->declare_parameter<double>("params/gate_ratio", 0.1);
  nh->get_parameter("params/gate_ratio", _map_gen_pa.gate_ratio_);
  nh->declare_parameter<double>("params/ellip_ratio", 0.1);
  nh->get_parameter("params/ellip_ratio", _map_gen_pa.ellip_ratio_);
  nh->declare_parameter<double>("params/poly_ratio", 0.1);
  nh->get_parameter("params/poly_ratio", _map_gen_pa.poly_ratio_);
  nh->declare_parameter<double>("params/w1", 0.3);
  nh->get_parameter("params/w1", _map_gen_pa.w1_);
  nh->declare_parameter<double>("params/w2", 1.0);
  nh->get_parameter("params/w2", _map_gen_pa.w2_);
  nh->declare_parameter<double>("params/w3", 2.0);
  nh->get_parameter("params/w3", _map_gen_pa.w3_);
  nh->declare_parameter<bool>("params/add_noise", false);
  nh->get_parameter("params/add_noise", _map_gen_pa.add_noise_);
  nh->declare_parameter<double>("params/seed", 1.0);
  nh->get_parameter("params/seed", _seed);

  nh->declare_parameter<bool>("dataset/save_map", false);
  nh->get_parameter("dataset/save_map", _save_map);
  nh->declare_parameter<int>("dataset/samples_num", 0);
  nh->get_parameter("dataset/samples_num", _samples_on_map);
  nh->declare_parameter<int>("dataset/start_index", 0);
  nh->get_parameter("dataset/start_index", _initial_num);
  nh->declare_parameter<std::string>("dataset/path", "path");
  nh->get_parameter("dataset/path", _dataset_path);

  nh->declare_parameter<std::string>("clear_path", "pos.csv");
  nh->get_parameter("clear_path", _clear_path);
  nh->declare_parameter<bool>("clear_pos", false);
  nh->get_parameter("clear_pos", _clear_pos);

  nh->declare_parameter<double>("dyn/v_x_h", 0.1);
  nh->declare_parameter<double>("dyn/v_y_h", 0.1);
  nh->declare_parameter<double>("dyn/v_z_h", 0.0);
  nh->declare_parameter<double>("dyn/dt", 10.0);
  nh->declare_parameter<bool>("dyn/dyn_mode", false);
  nh->get_parameter("dyn/v_x_h", _struct_map_gen.vel_h(0));
  nh->get_parameter("dyn/v_y_h", _struct_map_gen.vel_h(1));
  nh->get_parameter("dyn/v_z_h", _struct_map_gen.vel_h(2));
  nh->get_parameter("dyn/dt", dt);
  nh->get_parameter("dyn/dyn_mode", dyn_mode);

  if(!dyn_mode) readClearPos();

  rclcpp::Rate loop_rate(1.0 / dt);

  if (opendir(_dataset_path.c_str()) == NULL) {
    std::string cmd = "mkdir -p " + _dataset_path;
    system(cmd.c_str());
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  _struct_map_gen.initParams(_grid_mpa);
  _struct_map_gen.randomUniMapGen(_map_gen_pa, _seed, dyn_mode);
  _num += 1;
  pubSensedPoints();
  loop_rate.sleep();

  while (rclcpp::ok()) {
    if(!dyn_mode){
      if (_auto_gen && _num < _samples_on_map) {
        _seed += 1.0;
        _struct_map_gen.clear();
        _struct_map_gen.change_ratios(_seed, false, dt);

        _num += 1;
        pubSensedPoints();
      }
    }
    else{
      _struct_map_gen.clear();
      _struct_map_gen.dyn_generate(1/dt);
      pubSensedPoints();
    }
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}