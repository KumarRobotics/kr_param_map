#ifndef STRUCT_MAP_GEN_HPP
#define STRUCT_MAP_GEN_HPP

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

#include <map_utils/closed_shapes.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>



namespace param_env
{

  class StructMapGenerator
  {
  private:

    uniform_real_distribution<double> rand_w, rand_h, rand_cw, rand_radiu;

    param_env::GridMap grid_map_;
    param_env::GeoMap geo_map_;
    param_env::MapParams mpa;

    int _all_grids, _cylinder_grids, _circle_grids, _ellip_grids, _gate_grids, _poly_grids;

    double w1_, w2_, w3_, w4_;

    random_device rd;
    default_random_engine eng(rd());

    uniform_real_distribution<double> rand_theta;


    pcl::PointCloud<pcl::PointXYZ> cloudMap;


  public:

    GeoMapGenerator() = default;

    ~GeoMapGenerator() {}


    template<class T>
    int updatePts(T &geo_rep)
    {
      int cur_grids = 0;

      Eigen::Vector3d bound, cpt, ob_pt;
      geo_rep.getBd(bound);
      geo_rep.getCenter(cpt);

      int widNum1 = ceil(bound(0) / _resolution);
      int widNum2 = ceil(bound(1) / _resolution);
      int widNum3 = ceil(bound(2) / _resolution);

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
            if (!geo_rep.isInside(ob_pt))
            {
              continue;
            }
            _grid_map.setOcc(ob_pt);

            pcl::PointXYZ pt_random;
            pt_random.x = ob_pt(0);
            pt_random.y = ob_pt(1);
            pt_random.z = ob_pt(2);

            cloudMap.points.push_back(pt_random);
            cur_grids += 1;
          }
        }
      }  

      return cur_grids;
    }


    template<class T>
    void traversePts(std::vector<T> &geo_reps)
    {
      for(auto &geo_rep : geo_reps)
      {
        updatePts(geo_rep);
      }

    }


    //it should be called after the random map gene
    void resetRes(const float data){

      mpa.resolution_ = data;
      _grid_map.init(mpa);

      cloudMap.clear();

      std::vector<param_env::Polyhedron> polyhedron;
      std::vector<param_env::Cylinder> cylinder;
      std::vector<param_env::Ellipsoid> ellipsoid;
      std::vector<param_env::CircleGate> circle_gate;
      std::vector<param_env::RectGate> rect_gate; 

      _geo_map.getPolyhedron(polyhedron);
      _geo_map.getCylinder(cylinder);
      _geo_map.getEllipsoid(ellipsoid);
      _geo_map.getCircleGate(circle_gate);
      _geo_map.getRectGate(rect_gate);

      traversePts(polyhedron);
      traversePts(cylinder);
      traversePts(ellipsoid);
      traversePts(circle_gate);
      traversePts(rect_gate);
    }


  void randomUniMapGen()
  {
    Eigen::Vector3d bound;
    Eigen::Vector3d cpt; // center points, object points
    
    grid_map_.setUniRand(eng);

    rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);
    rand_w = uniform_real_distribution<double>(w1_, w2_);
    rand_h = uniform_real_distribution<double>(0.1, mpa.map_size_(2));
    rand_cw = uniform_real_distribution<double>(w1_, w3_);
    rand_radiu = uniform_real_distribution<double>(w1_, w4_);


    // generate cylinders
    int cur_grids = 0;
    while (cur_grids < _cylinder_grids)
    {
      grid_map_.getUniRandPos(cpt);

      double w, h;
      h = rand_h(eng);
      w = rand_cw(eng);

      param_env::Cylinder cylinder(cpt, w, h);

      cur_grids += updatePts(cylinder);
      geo_map_.add(cylinder);

    }
    _cylinder_grids = cur_grids;

    cur_grids = 0;
    // generate circle obs
    while (cur_grids < _circle_grids)
    {
      grid_map_.getUniRandPos(cpt);

      double theta = rand_theta(eng);
      double width = 0.1 + 0.2 * rand_radiu(eng);

      bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

      param_env::CircleGate cir_gate(cpt, bound, theta);

      // outdoor box: infl * radNum2 * radNum2
      // indoor box: infl * radNum1 * radNum1
      cur_grids += updatePts(cir_gate);
      geo_map_.add(cir_gate);
    }
    _circle_grids = cur_grids;

    cur_grids = 0;
    // generate circle obs
    while (cur_grids < _gate_grids)
    {

      grid_map_.getUniRandPos(cpt);

      double theta = rand_theta(eng);
      double width = 0.1 + 0.2 * rand_radiu(eng);

      bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

      param_env::RectGate rect_gate(cpt, bound, theta);

      cur_grids += updatePts(rect_gate);
      geo_map_.add(rect_gate);
    }
    _gate_grids = cur_grids;


    //std::cout <<  "_ellip_grids " << _ellip_grids << std::endl;
    // generate ellipsoid
    cur_grids = 0;
    while (cur_grids < _ellip_grids)
    {
      grid_map_.getUniRandPos(cpt);
      Eigen::Vector3d euler_angle;
      euler_angle << rand_theta(eng), rand_theta(eng), rand_theta(eng);
      bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);

      param_env::Ellipsoid ellip;
      ellip.init(cpt, bound, euler_angle);

      cur_grids += updatePts(ellip);
      geo_map_.add(ellip);
    }

    // generate polytopes
    cur_grids = 0;
    while (cur_grids < _poly_grids)
    {
      grid_map_.getUniRandPos(cpt);
      bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);

      param_env::Polyhedron poly;
      poly.randomInit(cpt, bound);

      cur_grids += updatePts(poly);
      geo_map_.add(poly);
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



    typedef shared_ptr<GeoMap> Ptr;
  


  };

}

#endif