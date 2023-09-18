#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <param_env_msgs/changeMap.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map_utils/geo_map.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/map_basics.hpp>
#include <map_utils/map_to_voxel.hpp>
#include <random>
#include <string>
// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

using namespace std;

std::string _frame_id;
ros::Publisher _all_cloud_pub, _all_map_pub;
ros::Subscriber _res_sub;
ros::ServiceServer _change_map_service;

int _mode;
sensor_msgs::PointCloud2 globalCloud_pcd, globalMap_pcd;
std::string topic_name;
/*** global params for cloudMap ***/
pcl::PointCloud<pcl::PointXYZ> cloudMap, gridCloudMap;

param_env::GridMapParams _grid_mpa;
param_env::GridMap _grid_map;
param_env::BasicMapParams _mpa;
double _inflate_radius = 0.0, _mav_radius = 0.1;
std::string _file_name;
std::filesystem::directory_iterator file_iter;
std::vector<std::filesystem::path> filenames;
int file_idx = 0;

bool _auto_gen = false, _use_folder = false, _publish_grid_centers = false,
     _evaluate = false;

//*** image params ***/
int _negate;
double _occ_th;

//*** ECI params ***/
double _density_index, _clutter_index, _structure_index, _seed;
Eigen::Vector3d _map_property;

ros::Publisher _voxel_no_inflation_map_pub, _voxel_map_pub;

void publishVoxelMap() {
  /**comment it for normal version without publish the voxel message**/
  kr_planning_msgs::VoxelMap voxel_map, voxel_no_inflation_map;

  //param_env::gridMapToVoxelMap(_grid_map, _frame_id, voxel_no_inflation_map);
  param_env::gridMapToInflaAndNoInflaVoxelMap(
      _grid_map, _frame_id, _inflate_radius, voxel_map, voxel_no_inflation_map);

  _voxel_map_pub.publish(voxel_map);
  _voxel_no_inflation_map_pub.publish(voxel_no_inflation_map);
  //std::cout << "publish the voxel map" << std::endl;
}

void toPcsMsg() {
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalCloud_pcd);
}

/*** read image map ***/
void read_img(std::string& path) {
  std::cout << path << std::endl;

  // cloudMap
  SDL_Surface* img;

  unsigned char* p;
  unsigned char value;
  int avg_channels, alpha, color_sum;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if (!(img = IMG_Load(path.c_str()))) {
    std::string errmsg = std::string("failed to open image file \"") +
                         std::string(path.c_str()) + std::string("\": ") +
                         IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Get values that we'll need to iterate through the pixels
  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;

  // Copy the image data into the map structure
  int dim_x = int(_mpa.map_size_(0) / _grid_mpa.resolution_);
  int dim_y = int(_mpa.map_size_(1) / _grid_mpa.resolution_);
  int dim_z = int(_mpa.map_size_(2) / _grid_mpa.resolution_);

  double ratio_map_to_img_x = img->w / dim_x;
  double ratio_map_to_img_y = img->h / dim_y;

  // std::cout << "dim_x" <<dim_x << std::endl;
  // std::cout << "dim_y" << dim_y << std::endl;

  // std::cout << "img->w" << img->w<< std::endl;
  // std::cout << "img->h" << img->h << std::endl;
  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  avg_channels = n_channels;

  Eigen::Vector3d pos;
  // Copy pixel data into the map structure
  unsigned char* pixels = (unsigned char*)(img->pixels);
  for (unsigned int m = 0; m < dim_z; m++) {
    for (unsigned int j = 0; j < dim_y; j++) {
      for (unsigned int i = 0; i < dim_x; i++) {
        int j_px = int(j * ratio_map_to_img_y);
        int i_px = int(i * ratio_map_to_img_x);
        // Compute mean of RGB for this pixel
        p = pixels + j_px * rowstride + i_px * n_channels;
        color_sum = 0;
        for (unsigned int k = 0; k < avg_channels; k++) color_sum += *(p + (k));
        double color_avg = color_sum / (double)avg_channels;

        if (n_channels == 1)
          alpha = 1;
        else
          alpha = *(p + n_channels - 1);

        if (_negate == 1) color_avg = 255 - color_avg;

        // If negate is true, we consider blacker pixels free, and whiter
        // pixels occupied.  Otherwise, it's vice versa.
        double occ = (255 - color_avg) / 255.0;
        // Apply thresholds to RGB means to determine occupancy values for
        // map.  Note that we invert the graphics-ordering of the pixels to
        // produce a map with cell (0,0) in the lower-left corner.
        if (occ > _occ_th) {
          _grid_map.indexToPos(Eigen::Vector3i(i, dim_y - j - 1, m), pos);
          cloudMap.points.push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
        }
      }
    }
  }

  SDL_FreeSurface(img);
  toPcsMsg();
}

/*** read ros bag ***/
template <class T>
void read_pcs_bag(std::string& path, std::string& topic, T& msg) {
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

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
  if (!find) ROS_WARN("Fail to find '%s' in '%s'", topic.c_str(), path.c_str());

  return;
}

/*** read pcd file ***/
void read_pcs_pcd(std::string& path) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloudMap) ==
      -1)  //* load the file
  {
    PCL_ERROR("Couldn't read pcd file \n");
  }

  pcl::PointXYZ minPt, maxPt, delta_change;
  pcl::getMinMax3D(cloudMap, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << " Max y: " << maxPt.y
            << " Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << " Min y: " << minPt.y
            << " Min z: " << minPt.z << std::endl;

  delta_change.x = _mpa.map_origin_(0) - minPt.x;
  delta_change.y = _mpa.map_origin_(1) - minPt.y;
  delta_change.z = _mpa.map_origin_(2) - minPt.z;

  for (auto& pt : cloudMap) {
    pt.x += delta_change.x;
    pt.y += delta_change.y;
    pt.z += delta_change.z;
  }

  std::cout << "cloudMap.size()" << cloudMap.size() << std::endl;

  toPcsMsg();
}

/*** randomly gen points  ***/
void gen_pcs(int num = 1000) {
  std::default_random_engine eng(_seed);

  std::uniform_real_distribution<double> x_range(
      _mpa.map_origin_(0), _mpa.map_size_(0) + _mpa.map_origin_(0));
  std::uniform_real_distribution<double> y_range(
      _mpa.map_origin_(1), _mpa.map_size_(1) + _mpa.map_origin_(1));
  std::uniform_real_distribution<double> z_range(
      _mpa.map_origin_(2), _mpa.map_size_(2) + _mpa.map_origin_(2));

  int loop_n = 0;

  while (loop_n < num) {
    float ax = x_range(eng);
    float ay = y_range(eng);
    float az = z_range(eng);
    cloudMap.points.push_back(pcl::PointXYZ(ax, ay, az));
    loop_n++;
  }

  toPcsMsg();
}

void gen_ECI_pcs() {
  std::default_random_engine eng(_seed);

  std::uniform_real_distribution<double> x_range(
      _mpa.map_origin_(0), _mpa.map_size_(0) + _mpa.map_origin_(0));
  std::uniform_real_distribution<double> y_range(
      _mpa.map_origin_(1), _mpa.map_size_(1) + _mpa.map_origin_(1));
  std::uniform_real_distribution<double> z_range(
      _mpa.map_origin_(2), _mpa.map_size_(2) + _mpa.map_origin_(2));

  _grid_map.setUniRand(eng);
  _grid_map.ECIgenerate(_density_index, _clutter_index, _structure_index);
  _grid_map.getObsPts(cloudMap);

  toPcsMsg();
}

void pubSensedPoints() {
  globalCloud_pcd.header.frame_id = _frame_id;
  _all_cloud_pub.publish(globalCloud_pcd);

  if (_publish_grid_centers) {
    _grid_map.fillMap(cloudMap, 0.1);
    _grid_map.publishMap(gridCloudMap);
    std::cout << "gridCloudMap.size()" << gridCloudMap.size() << std::endl;

    pcl::toROSMsg(gridCloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = _frame_id;
    _all_map_pub.publish(globalMap_pcd);
  }

  if (_evaluate) {
    _map_property = _grid_map.evaluateEnv(_mav_radius);
  }
}

void readMap(std::string file_path) {
  switch (_mode) {
    case 0:
      // gen_pcs();
      gen_ECI_pcs();
      _seed += 1;
      break;
    case 1: {
      read_img(file_path);
      break;
    }
    case 2: {
      sensor_msgs::PointCloud msg;
      read_pcs_bag(file_path, topic_name, msg);
      convertPointCloudToPointCloud2(msg, globalCloud_pcd);
      pcl::fromROSMsg(globalCloud_pcd, cloudMap);
      break;
    }
    case 3: {
      read_pcs_bag(file_path, topic_name, globalCloud_pcd);
      pcl::fromROSMsg(globalCloud_pcd, cloudMap);
      break;
    }
    case 4:
      read_pcs_pcd(file_path);
      break;
  }

  pubSensedPoints();
  return;
}

bool nextFile() {
  file_idx++;
  if (file_idx >= filenames.size()) {
    file_idx = 0;
    ROS_WARN("No more files to read! Starting from 0 again.");
  }

  cloudMap.clear();
  _grid_map.clearAllOcc();
  _file_name = filenames[file_idx].string();
  readMap(_file_name);

  return true;
}

void resCallback(const std_msgs::Float32& msg) {
  float res = msg.data;
  float inv_res = 1.0 / res;

  if (inv_res - float((int)inv_res) < 1e-6) {
    _grid_mpa.resolution_ = res;
    _grid_map.initMap(_grid_mpa);

    pubSensedPoints();
  } else {
    ROS_WARN("The resolution is not valid! Try a different one !");
  }
}

// ToDo: Change to a specific map specified by the message
bool genMapCallback(param_env_msgs::changeMap::Request& req,
                    param_env_msgs::changeMap::Response& res) {
  nextFile();
  res.density_index = _map_property(0);
  res.clutter_index = _map_property(1);
  res.structure_index = _map_property(2);
  res.file_name.data = _file_name;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "param_map");
  ros::NodeHandle nh("~");

  _all_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_gridmap", 1);

  _res_sub = nh.subscribe("change_res", 10, resCallback);
  _change_map_service = nh.advertiseService("change_map", genMapCallback);

  _voxel_map_pub =
      nh.advertise<kr_planning_msgs::VoxelMap>("/mapper/local_voxel_map", 1);
  _voxel_no_inflation_map_pub = nh.advertise<kr_planning_msgs::VoxelMap>(
      "/mapper/local_voxel_no_inflation_map", 1);

  nh.param("map/x_size", _mpa.map_size_(0), 40.0);
  nh.param("map/y_size", _mpa.map_size_(1), 40.0);
  nh.param("map/z_size", _mpa.map_size_(2), 5.0);
  nh.param("map/x_origin", _mpa.map_origin_(0), -20.0);
  nh.param("map/y_origin", _mpa.map_origin_(1), -20.0);
  nh.param("map/z_origin", _mpa.map_origin_(2), 0.0);

  nh.param("map/resolution", _grid_mpa.resolution_, 0.1);
  nh.param("map/frame_id", _frame_id, string("map"));
  nh.param("map/inflate_radius", _inflate_radius, 0.1);

  nh.param("map/auto_change", _auto_gen, false);
  nh.param("map/publish_grid_centers", _publish_grid_centers, false);

  nh.param("map/evaluate", _evaluate, false);
  nh.param("map/mav_radius", _mav_radius, 0.1);

  nh.param("params/density_index", _density_index, 0.1);
  nh.param("params/clutter_index", _clutter_index, 0.1);
  nh.param("params/structure_index", _structure_index, 0.1);
  nh.param("params/seed", _seed, 0.1);

  // set up basic parameters for grid map
  _grid_mpa.basic_mp_ = _mpa;
  _grid_mpa.basic_mp_.min_range_ = _grid_mpa.basic_mp_.map_origin_;
  _grid_mpa.basic_mp_.max_range_ =
      _grid_mpa.basic_mp_.map_origin_ + _grid_mpa.basic_mp_.map_size_;
  _grid_mpa.basic_mp_.map_volume_ = _grid_mpa.basic_mp_.map_size_(0) *
                                    _grid_mpa.basic_mp_.map_size_(1) *
                                    _grid_mpa.basic_mp_.map_size_(2);
  _grid_map.initMap(_grid_mpa);

  nh.param("bag_topic", topic_name, std::string("point_clouds_topic"));

  nh.param("img/negate", _negate, 0);
  nh.param("img/occ_th", _occ_th, 0.65);

  // map mode
  // 0 --- randomly generate
  // 1 --- image
  // 2 --- read the ros bag poind cloud 1
  // 3 --- read the ros bag poind cloud 2
  // 4 --- read pcd file
  nh.param("map/mode", _mode, 0);

  std::string file_path, folder_path;

  nh.param("folder_path", folder_path, std::string(""));
  nh.param("use_folder", _use_folder, false);

  ros::Duration(5.0).sleep();

  if (_use_folder) {
    file_iter = std::filesystem::directory_iterator(folder_path);
    readMap(file_iter->path());
    // new stuff so we read the names in order
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
      filenames.push_back(entry.path());
    }
    std::sort(filenames.begin(), filenames.end());

  } else {
    nh.param("file_path", file_path, std::string("path"));
    readMap(file_path);
  }

  ros::Rate loop_rate(5.0);
  bool success = true;

  while (ros::ok()) {
    if (_auto_gen && _use_folder && success) {
      success = nextFile();
    } else if (_auto_gen && _mode == 0) {
      cloudMap.clear();
      _grid_map.clearAllOcc();
      readMap(file_path);
    }

    ros::spinOnce();
    publishVoxelMap();
    loop_rate.sleep();
  }
}
