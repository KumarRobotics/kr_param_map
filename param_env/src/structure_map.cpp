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

#include <map_utils/closed_shapes.hpp>
#include <map_utils/grid_map.hpp>

using namespace std;

vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

std::string _frame_id;
ros::Publisher _all_map_cloud_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

random_device rd;
default_random_engine eng(rd());

uniform_real_distribution<double> rand_theta;

double _resolution;

int _all_grids, _cylinder_grids, _circle_grids, _ellip_grids, _gate_grids, _poly_grids;

double _w1, _w2, _w3, _w4;

// for normal cylinders
uniform_real_distribution<double> rand_w, rand_h, rand_cw, rand_radiu;

param_env::GridMap _grid_map;
param_env::MapParams mpa;

void updatePt(Eigen::Vector3d &pt)
{
  _grid_map.setOcc(pt);

  pcl::PointXYZ pt_random;
  pt_random.x = pt(0);
  pt_random.y = pt(1);
  pt_random.z = pt(2);

  cloudMap.points.push_back(pt_random);
}


void RandomUniMapGenerate()
{
  
  int cur_grids;
  Eigen::Vector3d bound;
  Eigen::Vector3d cpt, ob_pt; // center points, object points
  int widNum1, widNum2, widNum3;
  Eigen::Matrix3d R;

  _grid_map.setUniRand(eng);

  rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);
  rand_w = uniform_real_distribution<double>(_w1, _w2);
  rand_h = uniform_real_distribution<double>(0.1, mpa.map_size_(2));
  rand_cw = uniform_real_distribution<double>(_w1, _w3);
  rand_radiu = uniform_real_distribution<double>(_w1, _w4);

  // generate cylinders
  cur_grids = 0;
  while (cur_grids < _cylinder_grids)
  {

    _grid_map.getUniRandPos(cpt);

    double w, h;
    h = rand_h(eng);
    w = rand_cw(eng);

    int heiNum = ceil(h / _resolution);
    int widNum = ceil(w / _resolution);

    // std::cout <<  "x : " << x << std::endl;
    // std::cout <<  "y : " << y << std::endl;
    for (int r = -widNum; r < widNum; r++)
    {
      for (int s = -widNum; s < widNum; s++)
      {
        if (r * r + s * s > (widNum * widNum))
        {
          continue;
        }
        for (int t = -heiNum / 2; t < heiNum / 2; t++)
        {
          ob_pt = cpt + Eigen::Vector3d(r * _resolution,
                                        s * _resolution,
                                        t * _resolution);

          if (_grid_map.isOcc(ob_pt) != 0)
          {
            continue;
          }

          updatePt(ob_pt);
          
          cur_grids += 1;
        }
      }
    }
  }
  _cylinder_grids = cur_grids;

  cur_grids = 0;
  // generate circle obs
  while (cur_grids < _circle_grids)
  {
    _grid_map.getUniRandPos(cpt);

    double theta = rand_theta(eng);
    double width = 0.1 + 0.2 * rand_radiu(eng);

    bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

    param_env::CircleGate cir_gate(cpt, bound, theta);

    // get bounding box
    widNum1 = ceil((bound(0) + bound(1)) / _resolution);
    widNum2 = widNum1;
    widNum3 = ceil(bound(2) / _resolution);

    // outdoor box: infl * radNum2 * radNum2
    // indoor box: infl * radNum1 * radNum1
    for (int r = -widNum1; r < widNum1; r++)
    {
      for (int s = -widNum2; s < widNum2; s++)
      {
        for (int t = -widNum3; t < widNum3; t++)
        {

          ob_pt = cpt + Eigen::Vector3d(r * _resolution,
                                        s * _resolution,
                                        t * _resolution);

          if (_grid_map.isOcc(ob_pt) != 0)
          {
            continue;
          }

          if (!cir_gate.isInside(ob_pt))
          {
            continue;
          }

          updatePt(ob_pt);

          cur_grids += 1;
        }
      }
    }
  }
  _circle_grids = cur_grids;

  cur_grids = 0;
  // generate circle obs
  while (cur_grids < _gate_grids)
  {

    _grid_map.getUniRandPos(cpt);

    double theta = rand_theta(eng);
    double width = 0.1 + 0.2 * rand_radiu(eng);

    bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

    param_env::RectGate rect_gate(cpt, bound, theta);

    // get bounding box
    widNum1 = ceil((bound(0) + bound(1)) / _resolution);
    widNum2 = widNum1;
    widNum3 = ceil(bound(2) / _resolution);

    // outdoor box: infl * radNum2 * radNum2
    // indoor box: infl * radNum1 * radNum1
    for (int r = -widNum1; r < widNum1; r++)
    {
      for (int s = -widNum2; s < widNum2; s++)
      {
        for (int t = -widNum3; t < widNum3; t++)
        {

          ob_pt = cpt + Eigen::Vector3d(r * _resolution,
                                        s * _resolution,
                                        t * _resolution);

          if (!rect_gate.isInside(ob_pt))
          {
            continue;
          }

          if (_grid_map.isOcc(ob_pt) != 0)
          {
            continue;
          }

          updatePt(ob_pt);

          cur_grids += 1;
        }
      }
    }
  }
  _gate_grids = cur_grids;
  //std::cout <<  "_ellip_grids " << _ellip_grids << std::endl;
  // generate ellipsoid
  cur_grids = 0;
  while (cur_grids < _ellip_grids)
  {
    _grid_map.getUniRandPos(cpt);

    Eigen::Vector3d euler_angle;
    euler_angle << rand_theta(eng), rand_theta(eng), rand_theta(eng);
    R = param_env::eulerToRot(euler_angle);

    bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);

    widNum1 = ceil(bound(0) / _resolution);
    widNum2 = ceil(bound(1) / _resolution);
    widNum3 = ceil(bound(2) / _resolution);

    Eigen::Matrix3d coeff_mat;
    coeff_mat << bound(0), 0.0, 0.0,
                  0.0, bound(1), 0.0,
                  0.0, 0.0, bound(2);

    Eigen::Matrix3d E = R * coeff_mat * R.transpose();
    param_env::Ellipsoid ellip(E, cpt);

    for (int r = -widNum1; r < widNum1; r++)
    {
      for (int s = -widNum2; s < widNum2; s++)
      {
        for (int t = -widNum3; t < widNum3; t++)
        {
          ob_pt = cpt + Eigen::Vector3d(r * _resolution,
                                        s * _resolution,
                                        t * _resolution);

          if (_grid_map.isOcc(ob_pt) != 0)
          {
            continue;
          }

          if (!ellip.isInside(ob_pt))
          {
            // std::cout <<  "ellip.isInside is false " << std::endl;
            continue;
          }

          updatePt(ob_pt);

          cur_grids += 1;
        }
      }
    }
  }
  //std::cout <<  "_poly_grids " << _poly_grids << std::endl;
  // generate polytopes
  cur_grids = 0;
  while (cur_grids < _poly_grids)
  {
    _grid_map.getUniRandPos(cpt);

    bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);

    widNum1 = ceil(bound(0) / _resolution);
    widNum2 = ceil(bound(1) / _resolution);
    widNum3 = ceil(bound(2) / _resolution);

    param_env::Polyhedron poly;
    poly.randomInit(cpt, bound);

    for (int r = -widNum1; r < widNum1; r++)
    {
      for (int s = -widNum2; s < widNum2; s++)
      {
        for (int t = -widNum3; t < widNum3; t++)
        {

          ob_pt = cpt + Eigen::Vector3d(r * _resolution,
                                         s * _resolution,
                                         t * _resolution);

          if (_grid_map.isOcc(ob_pt) != 0)
          {
            //std::cout <<  "isOcc " << std::endl;
            continue;
          }
         
          // std::cout << ob_pt << std::endl;
          if (!poly.isInside(ob_pt))
          {
            //std::cout <<  "poly.isInside is false " << std::endl;
            continue;
          }

          updatePt(ob_pt);

          cur_grids += 1;
        }
      }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  std::cout << "Finished generate random map !" << std::endl;
  std::cout << "The space ratio for cylinders, circles, gates, ellipsoids, and polytopes are: " << std::endl;
  std::cout << setiosflags(ios::fixed) << setprecision(4) << std::endl;
  std::cout << float(_cylinder_grids) / float(_all_grids) << " ";
  std::cout << float(_circle_grids) / float(_all_grids) << " ";
  std::cout << float(_gate_grids) / float(_all_grids) << " ";
  std::cout << float(_ellip_grids) / float(_all_grids) << " ";
  std::cout << float(_poly_grids) / float(_all_grids) << std::endl;
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