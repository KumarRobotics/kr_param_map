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
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_cw;
uniform_real_distribution<double> rand_radiu;

param_env::GridMap _grid_map;
param_env::MapParams mpa;


void RandomMapGenerate()
{

  pcl::PointXYZ pt_random;
  geometry_msgs::Pose pt;
  pt.orientation.w = 1.0;
  int cur_grids;

  _grid_map.setUniRand(eng);

  rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);


  rand_w = uniform_real_distribution<double>(_w1, _w2);
  rand_h = uniform_real_distribution<double>(0.1, mpa.map_size_(2));
  rand_cw = uniform_real_distribution<double>(_w1, _w3);
  rand_radiu = uniform_real_distribution<double>(_w1, _w4);

  Eigen::Vector3d cp, cpt, cpt_if;


  // generate cylinders
  cur_grids = 0;
  while (cur_grids < _cylinder_grids)
  {
    

    _grid_map.getUniRandPos(cp);

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
        for (int t = - heiNum / 2; t < heiNum / 2; t++)
        {
          cpt << cp + Eigen::Vector3d(r * _resolution, 
                                      s * _resolution,
                                      t * _resolution);
          
          if (!_grid_map.isInMap(cpt))
          {
            continue;
          }

          pt_random.x = cpt(0);
          pt_random.y = cpt(1);
          pt_random.z = cpt(2);

          cloudMap.points.push_back(pt_random);
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
    _grid_map.getUniRandPos(cp);

    double theta = rand_theta(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0,
        sin(theta), cos(theta), 0.0,
        0, 0, 1;

    double radius1 = rand_radiu(eng);
    double radius2 = rand_radiu(eng);

    int widNum1  = ceil(rand_w(eng) / _resolution);
    int widNum2  = ceil(rand_w(eng) / _resolution);
    int widNum3  = ceil(rand_w(eng) / _resolution);


    // draw a circle centered at (x,y,z)

    for (double angle = 0.0; angle < 2 * M_PI; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      for (int ifx = -0; ifx <= widNum1 / 2; ++ifx)
      {
        for (int ify = -0; ify <= widNum2 / 2; ++ify)
        {
          for (int ifz = -0; ifz <= widNum3 / 2; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + cp;

            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);

            if (!_grid_map.isInMap(cpt_if))
            {
              continue;
            }

            

            cloudMap.push_back(pt_random);
            cur_grids += 1;
          }
        }
      }
    }
  }
  _circle_grids = cur_grids;

  cur_grids = 0;
  // generate circle obs
  while (cur_grids < _gate_grids)
  {

    _grid_map.getUniRandPos(cp);

    double theta = rand_theta(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0,
              sin(theta), cos(theta), 0.0,
              0, 0, 1;

    int radNum1 = ceil(rand_radiu(eng) / _resolution);
    int radNum2 = radNum1 + ceil(rand_w(eng) / _resolution);
    int widNum  = ceil(rand_w(eng) / _resolution);
    
    // outdoor box: infl * radNum2 * radNum2
    // indoor box: infl * radNum1 * radNum1
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    {
      for (int s = -radNum2; s < radNum2; s++)
      {
        for (int t = -radNum2; t < radNum2; t++)
        {
          if (abs(t) <= radNum1 & abs(s) <= radNum1)
          {
            continue;
          }          

          cpt << cp + rotate * Eigen::Vector3d(r * _resolution, 
                                               s * _resolution,
                                               t * _resolution);

          if (!_grid_map.isInMap(cpt))
          {
            continue;
          }


          pt_random.x = cpt(0);
          pt_random.y = cpt(1);
          pt_random.z = cpt(2);

          cloudMap.points.push_back(pt_random);
          cur_grids += 1;
        }
      }
    }
  }
  _gate_grids = cur_grids;


  // generate ellipsoid
  cur_grids = 0;
  while (cur_grids < _ellip_grids)
  {

    _grid_map.getUniRandPos(cp);

    Eigen::Vector3d euler_angle;
    euler_angle << rand_theta(eng), rand_theta(eng), rand_theta(eng);

    double l1, l2, l3;
    l1 = rand_radiu(eng);
    l2 = rand_radiu(eng);
    l3 = rand_radiu(eng);

    int l1_num = ceil(l1 / _resolution);
    int l2_num = ceil(l2 / _resolution);
    int l3_num = ceil(l3 / _resolution);

    Eigen::Matrix3d R = param_env::eulerToRot(euler_angle);

    Eigen::Matrix3d coeff_mat;
    coeff_mat << l1, 0.0, 0.0,
                 0.0, l2, 0.0,
                 0.0, 0.0, l3;

    Eigen::Matrix3d E = R * coeff_mat * R.transpose();
    param_env::Ellipsoid ellip(E, center_pt);

    //std::cout <<  "center_pt is : " << center_pt << std::endl;
    for (int r = -l1_num; r < l1_num; r++)
    {
      for (int s = -l2_num; s < l2_num; s++)
      {
        for (int t = -l3_num; t < l3_num; t++)
        {
          cpt = R * Eigen::Vector3d(r * _resolution,
                                    s * _resolution,
                                    t * _resolution) +  cp;
          //std::cout << cpt << std::endl;
          if (!ellip.isInside(cpt) | !_grid_map.isInMap(cpt))
          {
            //std::cout <<  "ellip.isInside is false " << std::endl;
            continue;
          }

          pt_random.x = cpt(0);
          pt_random.y = cpt(1);
          pt_random.z = cpt(2);

          cloudMap.points.push_back(pt_random);
          cur_grids += 1;
        }
      }
    }
  }


  // generate polytopes
  cur_grids = 0;
  while (cur_grids < _poly_grids)
  {
    _grid_map.getUniRandPos(cp);

    Eigen::Vector3d bound;
    bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);


    int l1_num = ceil(bound(0) / _resolution);
    int l2_num = ceil(bound(1) / _resolution);
    int l3_num = ceil(bound(2) / _resolution);

    param_env::Polyhedron poly;
    poly.randomInit(cp, bound);

    for (int r = -l1_num; r < l1_num; r++)
    {
      for (int s = -l2_num; s < l2_num; s++)
      {
        for (int t = -l3_num; t < l3_num; t++)
        {
          cpt = Eigen::Vector3d(r * _resolution,
                                s * _resolution,
                                t * _resolution) +  cp;
          //std::cout << cpt << std::endl;
          if (!poly.isInside(cpt) | !_grid_map.isInMap(cpt))
          {
            //std::cout <<  "ellip.isInside is false " << std::endl;
            continue;
          }

          pt_random.x = cpt(0);
          pt_random.y = cpt(1);
          pt_random.z = cpt(2);

          cloudMap.points.push_back(pt_random);
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
  std::cout<<setiosflags(ios::fixed)<<setprecision(4)<<std::endl;
  std::cout << float(_cylinder_grids)/float(_all_grids)<< " ";
  std::cout << float(_circle_grids)/float(_all_grids)<< " ";
  std::cout << float(_gate_grids)/float(_all_grids)<< " ";
  std::cout << float(_ellip_grids)/float(_all_grids)<< " ";
  std::cout << float(_poly_grids)/float(_all_grids) << std::endl;


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
  _all_grids = _map_size(0) * _map_size(1) * _map_size(2) / std::pow(_resolution, 3);

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


  RandomMapGenerate();

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}