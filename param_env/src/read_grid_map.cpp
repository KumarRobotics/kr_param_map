#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/logging_macros.h>

#include <Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map_utils/geo_map.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/map_basics.hpp>
#include <random>
// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

using namespace std;

std::string _frame_id;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_cloud_pub, _all_map_pub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _res_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _gen_map_sub;
int _mode;
sensor_msgs::msg::PointCloud2 globalCloud_pcd, globalMap_pcd;
std::string topic_name;
/*** global params for cloudMap ***/
pcl::PointCloud<pcl::PointXYZ> cloudMap, gridCloudMap;

param_env::GridMapParams _grid_mpa;
param_env::GridMap _grid_map;
param_env::BasicMapParams _mpa;
double _inflate_radius = 0.0;
std::filesystem::directory_iterator file_iter;
bool _auto_gen = false, _use_folder = false, _publish_grid_centers = false;

//*** image params ***/
int _negate;
double _occ_th;

void toPcsMsg() {
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalCloud_pcd);
}

/*** read image map ***/
void read_img(std::string& path) {
  std::cout << "Reading image map from: " << path << std::endl;

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

  std::cout << "cloudMap size: " << cloudMap.size() << std::endl;
  SDL_FreeSurface(img);
  toPcsMsg();
}

template <class T>
void read_pcs_bag(const std::string & path, const std::string & topic, T & msg)
{
  rosbag2_cpp::Reader reader;
  reader.open(path);

  rclcpp::Serialization<T> serializer;
  bool found = false;

  while (reader.has_next())
  {
    auto bag_message = reader.read_next();

    if (bag_message->topic_name == topic)
    {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      T tmp_msg;
      serializer.deserialize_message(&serialized_msg, &tmp_msg);

      msg = tmp_msg;
      RCUTILS_LOG_WARN("Get data!");
      found = true;
      break;
    }
  }

  if (!found) {
    RCUTILS_LOG_WARN("Fail to find '%s' in '%s'", topic.c_str(), path.c_str());
  }
}

/*** read pcd file ***/
void read_pcs_pcd(std::string& path) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloudMap) ==
      -1)  //* load the file
  {
    PCL_ERROR("Couldn't read pcd file \n");
  }

  toPcsMsg();
}

/*** randomly gen points  ***/
void gen_pcs(float bound = 50, int num = 10000) {
  std::default_random_engine random(time(NULL));
  std::uniform_real_distribution<double> r(-bound, bound);
  int loop_n = 0;

  while (loop_n < num) {
    float ax = r(random);
    float ay = r(random);
    float az = r(random);
    float fx1 = bound / 4.0;
    float fy1 = bound / 4.0;
    float fy2 = bound / 4.0;
    float fz2 = bound / 4.0;
    if (ax < fx1 && ax > -fx1 && ay < fy1 && ay > -fy1) continue;
    if (ay < fy2 && ay > -fy2 && az < fz2 && az > -fz2) continue;
    cloudMap.points.push_back(pcl::PointXYZ(ax, ay, az));
    loop_n++;
  }

  toPcsMsg();
}

void pubSensedPoints() {
  globalCloud_pcd.header.frame_id = _frame_id;
  _all_cloud_pub->publish(globalCloud_pcd);

  if (_publish_grid_centers) {
    _grid_map.fillMap(cloudMap, _inflate_radius);
    _grid_map.publishMap(gridCloudMap);

    pcl::toROSMsg(gridCloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = _frame_id;
    _all_map_pub->publish(globalMap_pcd);
  }
}

void readMap(std::string file_path) {
  switch (_mode) {
    case 0:
      gen_pcs();
      break;
    case 1: {
      read_img(file_path);
      break;
    }
    case 2: {
      sensor_msgs::msg::PointCloud msg;
      read_pcs_bag(file_path, topic_name, msg);
      sensor_msgs::convertPointCloudToPointCloud2(msg, globalCloud_pcd);
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
  file_iter++;
  if (file_iter == std::filesystem::directory_iterator()) {
    std::cout << "No more files in the folder!" << std::endl;
    return false;
  }
  cloudMap.clear();
  _grid_map.clearAllOcc();
  readMap(file_iter->path());
  return true;
}

void resCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  float res = msg->data;
  float inv_res = 1.0 / res;

  if (inv_res - float((int)inv_res) < 1e-6) {
    _grid_mpa.resolution_ = res;
    _grid_map.initMap(_grid_mpa);
    pubSensedPoints();
  } else {
    RCLCPP_WARN(rclcpp::get_logger("param_map"), "The resolution is not valid! Try a different one!");
  }
}

void genMapCallback(const std_msgs::msg::Bool::SharedPtr msg) { nextFile(); }


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("param_map");

  // Publishers
  _all_cloud_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_cloud", 1);
  _all_map_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("global_gridmap", 1);

  // Subscribers
  _res_sub = nh->create_subscription<std_msgs::msg::Float32>(
      "change_res", 10, resCallback);
  _gen_map_sub = nh->create_subscription<std_msgs::msg::Bool>(
      "change_map", 10, genMapCallback);

  // Parameters (declare then get)
  nh->declare_parameter("map/x_size", 40.0);
  nh->declare_parameter("map/y_size", 40.0);
  nh->declare_parameter("map/z_size", 5.0);
  nh->declare_parameter("map/x_origin", -20.0);
  nh->declare_parameter("map/y_origin", -20.0);
  nh->declare_parameter("map/z_origin", 0.0);
  nh->declare_parameter("map/resolution", 0.1);
  nh->declare_parameter("map/frame_id", "map");
  nh->declare_parameter("map/inflate_radius", 0.1);
  nh->declare_parameter("map/auto_change", false);
  nh->declare_parameter("map/publish_grid_centers", false);
  nh->declare_parameter("bag_topic", "point_clouds_topic");
  nh->declare_parameter("img/negate", 0);
  nh->declare_parameter("img/occ_th", 0.65);
  nh->declare_parameter("map/mode", 0);
  nh->declare_parameter("folder_path", std::string(""));
  nh->declare_parameter("use_folder", false);
  nh->declare_parameter("file_path", std::string("path"));

  nh->get_parameter("map/x_size", _mpa.map_size_(0));
  nh->get_parameter("map/y_size", _mpa.map_size_(1));
  nh->get_parameter("map/z_size", _mpa.map_size_(2));
  nh->get_parameter("map/x_origin", _mpa.map_origin_(0));
  nh->get_parameter("map/y_origin", _mpa.map_origin_(1));
  nh->get_parameter("map/z_origin", _mpa.map_origin_(2));
  nh->get_parameter("map/resolution", _grid_mpa.resolution_);
  nh->get_parameter("map/frame_id", _frame_id);
  nh->get_parameter("map/inflate_radius", _inflate_radius);
  nh->get_parameter("map/auto_change", _auto_gen);
  nh->get_parameter("map/publish_grid_centers", _publish_grid_centers);
  nh->get_parameter("bag_topic", topic_name);
  nh->get_parameter("img/negate", _negate);
  nh->get_parameter("img/occ_th", _occ_th);

  // map mode
  // 0 --- randomly generate
  // 1 --- image
  // 2 --- read the ros bag poind cloud 1
  // 3 --- read the ros bag poind cloud 2
  // 4 --- read pcd file
  nh->get_parameter("map/mode", _mode);
  std::string folder_path;
  nh->get_parameter("folder_path", folder_path);
  nh->get_parameter("use_folder", _use_folder);
  std::string file_path;
  nh->get_parameter("file_path", file_path);

  // Init grid map params
  _grid_mpa.basic_mp_ = _mpa;
  _grid_mpa.basic_mp_.min_range_ = _grid_mpa.basic_mp_.map_origin_;
  _grid_mpa.basic_mp_.max_range_ =
      _grid_mpa.basic_mp_.map_origin_ + _grid_mpa.basic_mp_.map_size_;
  _grid_mpa.basic_mp_.map_volume_ = _grid_mpa.basic_mp_.map_size_(0) *
                                    _grid_mpa.basic_mp_.map_size_(1) *
                                    _grid_mpa.basic_mp_.map_size_(2);
  _grid_map.initMap(_grid_mpa);

  rclcpp::sleep_for(std::chrono::seconds(5));

  if (_use_folder) {
    file_iter = std::filesystem::directory_iterator(folder_path);
    readMap(file_iter->path());
  } else {
    readMap(file_path);
  }

  rclcpp::Rate loop_rate(10);

  bool success = true;
  while (rclcpp::ok()) {
    if (_auto_gen && _use_folder && success) {
      success = nextFile();
    }

    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
