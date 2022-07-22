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

  struct MapGenParams
  {
    /* parameters for map generator */
    double cylinder_ratio_, circle_ratio_, gate_ratio_, ellip_ratio_, poly_ratio_;
    double w1_, w2_, w3_, w4_;
  };
  
  class StructMapGenerator
  {
  private:

    pcl::PointCloud<pcl::PointXYZ> cloudMap_;

    param_env::GridMap grid_map_;
    param_env::GeoMap geo_map_;
    param_env::MapGenParams mgpa_;
    double map_resolution_;
    double map_volume_;

    random_device rd;
    uniform_real_distribution<double> rand_theta, rand_w, rand_h, rand_cw, rand_radiu;

  public:

    StructMapGenerator() = default;

    ~StructMapGenerator() {}

    template<class T>
    int updatePts(T &geo_rep)
    {
      int cur_grids = 0;

      Eigen::Vector3d bound, cpt, ob_pt;
      geo_rep.getBd(bound);
      geo_rep.getCenter(cpt);

      int widNum1 = ceil(bound(0) / map_resolution_);
      int widNum2 = ceil(bound(1) / map_resolution_);
      int widNum3 = ceil(bound(2) / map_resolution_);

      for (int r = -widNum1; r < widNum1; r++)
      {
        for (int s = -widNum2; s < widNum2; s++)
        {
          for (int t = -widNum3; t < widNum3; t++)
          {
            ob_pt = cpt + Eigen::Vector3d(r * map_resolution_,
                                          s * map_resolution_,
                                          t * map_resolution_);
                           
            if (grid_map_.isOcc(ob_pt) != 0)
            {
              continue;
            }


            if (!geo_rep.isInside(ob_pt))
            {
              continue;
            }

            
            grid_map_.setOcc(ob_pt);

            pcl::PointXYZ pt_random;
            pt_random.x = ob_pt(0);
            pt_random.y = ob_pt(1);
            pt_random.z = ob_pt(2);

            cloudMap_.points.push_back(pt_random);
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

    void initParams(param_env::MapParams &mpa)
    {
      grid_map_.initMap(mpa);
      map_resolution_ = mpa.resolution_;
      map_volume_ = mpa.map_size_(0)*mpa.map_size_(1)*mpa.map_size_(2);

      rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);
      rand_h = uniform_real_distribution<double>(0.1, mpa.map_size_(2));
    }

    
    void getPC(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      cloudMap_.width = cloudMap_.points.size();
      cloudMap_.height = 1;
      cloudMap_.is_dense = true;

      cloudMap = cloudMap_;

    }


    //it should be called after the random map gene
    void resetMap(){

      cloudMap_.clear();

      std::vector<param_env::Polyhedron> polyhedron;
      std::vector<param_env::Cylinder> cylinder;
      std::vector<param_env::Ellipsoid> ellipsoid;
      std::vector<param_env::CircleGate> circle_gate;
      std::vector<param_env::RectGate> rect_gate; 

      geo_map_.getPolyhedron(polyhedron);
      geo_map_.getCylinder(cylinder);
      geo_map_.getEllipsoid(ellipsoid);
      geo_map_.getCircleGate(circle_gate);
      geo_map_.getRectGate(rect_gate);

      traversePts(polyhedron);
      traversePts(cylinder);
      traversePts(ellipsoid);
      traversePts(circle_gate);
      traversePts(rect_gate);
    }


    void randomUniMapGen(param_env::MapGenParams &mgpa)
    {
      Eigen::Vector3d bound;
      Eigen::Vector3d cpt; // center points, object points

      mgpa_ = mgpa;

      default_random_engine eng(rd());
      
      grid_map_.setUniRand(eng);

      int all_grids = map_volume_ / std::pow(map_resolution_, 3);
      int cylinder_grids = ceil(all_grids * mgpa_.cylinder_ratio_);
      int circle_grids   = ceil(all_grids * mgpa_.circle_ratio_);
      int gate_grids     = ceil(all_grids * mgpa_.gate_ratio_);
      int ellip_grids    = ceil(all_grids * mgpa_.ellip_ratio_);
      int poly_grids     = ceil(all_grids * mgpa_.poly_ratio_);

      rand_w = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w2_);
      rand_cw = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w3_);
      rand_radiu = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w4_);


      // generate cylinders
      int cur_grids = 0;
      double w, h;

      while (cur_grids < cylinder_grids)
      {
        grid_map_.getUniRandPos(cpt);
       
        
        h = rand_h(eng);
        w = rand_cw(eng);

        param_env::Cylinder cylinder(cpt, w, h);

        cur_grids += updatePts(cylinder);
        geo_map_.add(cylinder);

      }
      cylinder_grids = cur_grids;

      cur_grids = 0;
      // generate circle obs
      while (cur_grids < circle_grids)
      {
        grid_map_.getUniRandPos(cpt);

        double theta = rand_theta(eng);
        double width = 0.1 + 0.2 * rand_radiu(eng);

        bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

        param_env::CircleGate cir_gate(cpt, bound, theta);

        cur_grids += updatePts(cir_gate);
        geo_map_.add(cir_gate);
      }
      circle_grids = cur_grids;

      cur_grids = 0;
      // generate circle obs
      while (cur_grids < gate_grids)
      {

        grid_map_.getUniRandPos(cpt);

        double theta = rand_theta(eng);
        double width = 0.1 + 0.2 * rand_radiu(eng);

        bound << width, width + rand_radiu(eng), width + rand_radiu(eng);

        param_env::RectGate rect_gate(cpt, bound, theta);

        cur_grids += updatePts(rect_gate);
        geo_map_.add(rect_gate);
      }
      gate_grids = cur_grids;


      //std::cout <<  "ellip_grids " << ellip_grids << std::endl;
      // generate ellipsoid
      cur_grids = 0;
      while (cur_grids < ellip_grids)
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
      while (cur_grids < poly_grids)
      {
        grid_map_.getUniRandPos(cpt);
        bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);

        param_env::Polyhedron poly;
        poly.randomInit(cpt, bound);

        cur_grids += updatePts(poly);
        geo_map_.add(poly);
      }

      std::cout << setiosflags(ios::fixed) << setprecision(2) << std::endl;
      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;
      std::cout << "+++ Finished generate random map ! +++" << std::endl;
      std::cout << "+++ The ratios for geometries are: +++" << std::endl;
      std::cout << "+++ cylinders  : " << 100 * float(cylinder_grids) / float(all_grids) << "%           +++" << std::endl;
      std::cout << "+++ circles    : " << 100 * float(circle_grids) / float(all_grids)   << "%           +++" << std::endl;
      std::cout << "+++ gates      : " << 100 * float(gate_grids) / float(all_grids)     << "%           +++" << std::endl;
      std::cout << "+++ ellipsoids : " << 100 * float(ellip_grids) / float(all_grids)    << "%           +++" << std::endl;
      std::cout << "+++ polytopes  : " << 100 * float(poly_grids) / float(all_grids)     << "%           +++" << std::endl;    
      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;
    }


  };

}

#endif