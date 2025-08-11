#ifndef STRUCT_MAP_GEN_HPP
#define STRUCT_MAP_GEN_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <iterator>
#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map_utils/geo_map.hpp>
#include <map_utils/grid_map.hpp>
#include <map_utils/map_basics.hpp>
#include <random>
#include <pcl/common/transforms.h>

namespace param_env {

  struct MapGenParams
  {
    /* parameters for map generator */
    double cylinder_ratio_, circle_ratio_, gate_ratio_, ellip_ratio_, poly_ratio_;
    double w1_, w2_, w3_;
    bool add_noise_ = false;
  };
  
  class StructMapGenerator
  {
  private:

    pcl::PointCloud<pcl::PointXYZ> cloudMap_;

    param_env::GridMap grid_map_;
    param_env::GeoMap geo_map_;

    param_env::GridMapParams mpa_;
    param_env::MapGenParams mgpa_;

    default_random_engine eng;
    uniform_real_distribution<double> rand_theta, rand_w, rand_h, rand_cw, rand_radiu, rand_vx, rand_vy, rand_vz;

    // dyn param
    std::vector<param_env::Cylinder> cyl_list; std::vector<param_env::CircleGate>  cir_list;
    std::vector<param_env::RectGate>  gate_list; std::vector<param_env::Ellipsoid>  ellip_list;
    std::vector<param_env::Polyhedron> poly_list; 

  public:

    Eigen::Vector3d vel, vel_l, vel_h;

    StructMapGenerator() = default;
    ~StructMapGenerator() {}

    void getGridMap(param_env::GridMap& grid_map)
    {
      grid_map = grid_map_;
    }

    template<class T>
    int updatePts(T &geo_rep)
    {
      int cur_grids = 0;

      Eigen::Vector3d bound, cpt, ob_pt;
      geo_rep.getBd(bound);
      geo_rep.getCenter(cpt);

      int widNum1 = ceil(bound(0) / mpa_.resolution_);
      int widNum2 = ceil(bound(1) / mpa_.resolution_);
      int widNum3 = ceil(bound(2) / mpa_.resolution_);

      std::vector<Eigen::Vector3d> total_pts;

      for (int r = -widNum1; r < widNum1; r++)
        for (int s = -widNum2; s < widNum2; s++)
          for (int t = -widNum3; t < widNum3; t++)
          {
            ob_pt = cpt + Eigen::Vector3d(r * mpa_.resolution_,
                                          s * mpa_.resolution_,
                                          t * mpa_.resolution_);      
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
            total_pts.push_back(ob_pt);

            geo_rep.cloud->push_back(pt_random); // get cloud at 1st iter
          }

      if (mgpa_.add_noise_)
      {
        // add randorm noise
        std::normal_distribution<double> dist(0.0, mgpa_.w1_);
          
        for (auto ob_pt: total_pts)
        {
          double ran = (float) rand()/RAND_MAX; 
          if (ran < mgpa_.w1_)
          {
            ob_pt(0) = ob_pt(0) + dist(eng);
            ob_pt(1) = ob_pt(1) + dist(eng);
            ob_pt(2) = ob_pt(2) + dist(eng);


            if (!grid_map_.isInMap(ob_pt))
            {
              continue;
            }
            grid_map_.setOcc(ob_pt);
            pcl::PointXYZ pt_random;
            
            pt_random.x = ob_pt(0);
            pt_random.y = ob_pt(1);
            pt_random.z = ob_pt(2);
            cloudMap_.points.push_back(pt_random);

            geo_rep.cloud->push_back(pt_random); // get cloud at 1st iter
          }
        }

      }
      return cur_grids;
    }

    template<class T>
    int updatePtsLight(T &geo_rep)
    {
      int cur_grids = 0;
      Eigen::Vector3d rect, cpt, ob_pt;
      geo_rep.getRect(rect); //width l1 l2
      geo_rep.getCenter(cpt);
      Eigen::Matrix3d rot;
      geo_rep.getRot(rot);

      int widNum1 = ceil(rect(1) / mpa_.resolution_);
      int widNum2 = ceil(rect(2) / mpa_.resolution_);
      int width   = ceil(rect(0) / mpa_.resolution_);
      std::vector<Eigen::Vector3d> total_pts;
      Eigen::MatrixXd signs(2, 4);
      signs <<  1, -1, 1, -1,
                1, 1, -1, -1;
      for (int t = - width; t < width; t++)
        for (int r = 0 ; r < widNum1; r++)
          for (int s = 0; s < widNum2; s++)
          {
            for (int i = 0; i < 4; i++)
            {
              
              ob_pt = cpt + rot * Eigen::Vector3d(t * mpa_.resolution_,
                                            r * signs(0, i) * mpa_.resolution_,
                                            s * signs(1, i) * mpa_.resolution_);

              if (!geo_rep.isInside(ob_pt))
              {
                continue;
              }
              if (grid_map_.isOcc(ob_pt) != 0)
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
              total_pts.push_back(ob_pt);

              geo_rep.cloud->push_back(pt_random); // get cloud at 1st iter
            }
          }       
  

      if (mgpa_.add_noise_)
      {
        // add randorm noise
        std::normal_distribution<double> dist(0.0, mgpa_.w1_);
          
        for (auto ob_pt: total_pts)
        {
          double ran = (float) rand()/RAND_MAX; 
          if (ran < mgpa_.w1_)
          {
            ob_pt(0) = ob_pt(0) + dist(eng);
            ob_pt(1) = ob_pt(1) + dist(eng);
            ob_pt(2) = ob_pt(2) + dist(eng);
            if (!grid_map_.isInMap(ob_pt))
            {
              continue;
            }
            grid_map_.setOcc(ob_pt);
            pcl::PointXYZ pt_random;
            pt_random.x = ob_pt(0);
            pt_random.y = ob_pt(1);
            pt_random.z = ob_pt(2);
            cloudMap_.points.push_back(pt_random);

            geo_rep.cloud->push_back(pt_random); // get cloud at 1st iter
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

    void initParams(param_env::GridMapParams &mpa)
    {
      // update basic map paramaters
      mpa.basic_mp_.min_range_  = mpa.basic_mp_.map_origin_;
      mpa.basic_mp_.max_range_  = mpa.basic_mp_.map_origin_ + mpa.basic_mp_.map_size_;
      mpa.basic_mp_.map_volume_ = mpa.basic_mp_.map_size_(0)*mpa.basic_mp_.map_size_(1)*mpa.basic_mp_.map_size_(2);

      //count time 
      grid_map_.initMap(mpa);

      mpa_ = mpa;

      rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);
    }

    
    void changeRes(double &res)
    {
      mpa_.resolution_ = res;

      grid_map_.initMap(mpa_);

    }


    void getPC(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      cloudMap_.width = cloudMap_.points.size();
      cloudMap_.height = 1;
      cloudMap_.is_dense = true;

      cloudMap = cloudMap_;

    }

    void getPC2D(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      grid_map_.publish2dMap(cloudMap);

    }

    void clear(){
      cloudMap_.clear();
      grid_map_.clearAllOcc();
      geo_map_.clearAll();
    }

    //it should be called after the random map gene
    void resetMap()
    {

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
    
    void change_ratios(double &seed, bool if_dyn, float dt)
    {
      if(if_dyn) {  // dynamic update
        dyn_generate(dt);
        return;
      }

      eng.seed(seed);

      mgpa_.cylinder_ratio_ = mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.circle_ratio_   = 0.1  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.gate_ratio_     = 0.1  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.ellip_ratio_    = 0.5  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.poly_ratio_     = 0.5  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);

      generate(if_dyn); // false
    }

    void randomUniMapGen(param_env::MapGenParams &mgpa, double &seed, bool if_dyn)
    {

      mgpa_ = mgpa;

      rand_w     = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w2_);
      rand_radiu = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w3_);
      rand_h     = uniform_real_distribution<double>(mgpa_.w2_, mpa_.basic_mp_.map_size_(2));
      eng.seed(seed);
      generate(if_dyn);
    }

    void generate(bool dyn_mode)
    {
      // rand vel for each obs
      if(dyn_mode){
        rand_vx = uniform_real_distribution<double>(-vel_h(0), vel_h(0));
        rand_vy = uniform_real_distribution<double>(-vel_h(1), vel_h(1));
        rand_vz = uniform_real_distribution<double>(-vel_h(2), vel_h(2));
      }

      grid_map_.setUniRand(eng);

      int all_grids = ceil(mpa_.basic_mp_.map_volume_ / std::pow(mpa_.resolution_, 3));
      int cylinder_grids = ceil(all_grids * mgpa_.cylinder_ratio_);
      int circle_grids   = ceil(all_grids * mgpa_.circle_ratio_);
      int gate_grids     = ceil(all_grids * mgpa_.gate_ratio_);
      int ellip_grids    = ceil(all_grids * mgpa_.ellip_ratio_);
      int poly_grids     = ceil(all_grids * mgpa_.poly_ratio_);

      Eigen::Vector3d bound;
      Eigen::Vector3d cpt; // center points, object points

      // generate cylinders
      int cur_grids = 0;
      double w, h;
      while (cur_grids < cylinder_grids)
      {
        grid_map_.getUniRandPos(cpt);
       
        h = rand_h(eng);
        w = 0.2 + rand_w(eng);
        param_env::Cylinder cylinder(cpt, w, h);

        vel(0) = rand_vx(eng); vel(1) = rand_vy(eng); vel(2) = 0;
        cylinder.setVel(vel);
        cur_grids += updatePts(cylinder); // update pcl
        geo_map_.add(cylinder);

        if(dyn_mode){
          cyl_list.push_back(cylinder);  // add dyn_obs_list
        }
      }
      cylinder_grids = cur_grids;

      cur_grids = 0;
      // generate circle obs
      while (cur_grids < circle_grids)
      {
        grid_map_.getUniRandPos(cpt);

        double theta = rand_theta(eng);
        double width = 0.1 + 0.2 * rand_radiu(eng); // the half width
        bound << width, width + rand_radiu(eng), width + rand_radiu(eng);
        param_env::CircleGate cir_gate(cpt, bound, theta);

        vel(0) = rand_vx(eng); vel(1) = rand_vy(eng); vel(2) = 0;
        cir_gate.setVel(vel);
        cur_grids += updatePtsLight(cir_gate);
        geo_map_.add(cir_gate);

        if(dyn_mode){
          cir_list.push_back(cir_gate); // add dyn_obs_list
        }
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

        vel(0) = rand_vx(eng); vel(1) = rand_vy(eng); vel(2) = 0;
        rect_gate.setVel(vel);
        cur_grids += updatePtsLight(rect_gate);
        geo_map_.add(rect_gate);

        if(dyn_mode){
          gate_list.push_back(rect_gate); // add dyn_obs_list
        }
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

        vel(0) = rand_vx(eng); vel(1) = rand_vy(eng); vel(2) = 0;
        ellip.setVel(vel);
        cur_grids += updatePts(ellip);
        geo_map_.add(ellip);
        if(dyn_mode){
          ellip_list.push_back(ellip); // add dyn_obs_list
        }
      }
      ellip_grids = cur_grids;


      // generate polytopes
      cur_grids = 0;
      while (cur_grids < poly_grids)
      {
        grid_map_.getUniRandPos(cpt);
        bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng);
        param_env::Polyhedron poly;
        poly.randomInit(cpt, bound);

        vel(0) = rand_vx(eng); vel(1) = rand_vy(eng); vel(2) = 0;
        poly.setVel(vel);
        cur_grids += updatePts(poly); 
        geo_map_.add(poly);
        if(dyn_mode){
          poly_list.push_back(poly); // add dyn_obs_list
        }      
      }
      poly_grids = cur_grids;


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



    // for obs pointclouds transform
    template<class T>
    void move_clouds(T & geo_rep, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_out, Eigen::Vector3d & dist){
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << dist(0), dist(1), dist(2);
      transform.rotate(Eigen::Quaternionf::Identity());
      pcl::transformPointCloud(*(geo_rep.cloud), *cloud_ptr_out, transform);
    }


    // update grid map
    void pcl2grid(){
      Eigen::Vector3d pt = Eigen::Vector3d::Zero(3,1);
      int count = 0;
      for (auto ob_pt: cloudMap_.points)
        {
          pt(0) = ob_pt.x; pt(1) = ob_pt.y; pt(2) = ob_pt.z;
          if (!grid_map_.isInMap(pt))
          {
            continue;
          }
          grid_map_.setOcc(pt);
          count++;
        }
      std::cout << "GRID OBS: " << count << std::endl;
    }

    // Helper function to update dynamic object movement and point cloud
    template <typename ShapeT>
    void update_moving_shape(ShapeT &c, double dt,
                            double x_l, double x_h,
                            double y_l, double y_h,
                            double z_l, double z_h,
                            pcl::PointCloud<pcl::PointXYZ> &cloudMap_)
    {
        Eigen::Vector3d cur_cpt = Eigen::Vector3d::Zero();
        Eigen::Vector3d next_cpt = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d dist = Eigen::Vector3d::Zero();

        c.getCenter(cur_cpt);
        c.getVel(vel);

        // Update position based on velocity and time step
        next_cpt = cur_cpt + dt * vel;
        next_cpt(2) = cur_cpt(2); // Z stays constant

        // Boundary check and bounce
        auto bounce_axis = [](double &pos, double &vel, double min_v, double max_v) {
            if (pos < min_v) { pos = min_v; vel *= -1; }
            if (pos > max_v) { pos = max_v; vel *= -1; }
        };
        bounce_axis(next_cpt(0), vel(0), x_l, x_h);
        bounce_axis(next_cpt(1), vel(1), y_l, y_h);
        bounce_axis(next_cpt(2), vel(2), z_l, z_h);

        // Apply updated position and velocity
        c.setCenter(next_cpt);
        c.setVel(vel);

        // Move associated point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
        dist = next_cpt - cur_cpt;
        move_clouds(c, cloud_obs, dist);

        *(c.cloud) = *cloud_obs;   // update pcl
        cloudMap_ += *cloud_obs;   // add pcl to global map
    }

    // Main dynamic movement update function
    void dyn_generate(double dt)
    {
        // Movement bounds
        double x_l = -10, y_l = -10, z_l = 0;
        double x_h = 10,  y_h = 10,  z_h = 5;

        // Update all shape lists
        for (auto &c : cir_list)   update_moving_shape(c, dt, x_l, x_h, y_l, y_h, z_l, z_h, cloudMap_);
        for (auto &c : ellip_list) update_moving_shape(c, dt, x_l, x_h, y_l, y_h, z_l, z_h, cloudMap_);
        for (auto &c : cyl_list)   update_moving_shape(c, dt, x_l, x_h, y_l, y_h, z_l, z_h, cloudMap_);
        for (auto &c : poly_list)  update_moving_shape(c, dt, x_l, x_h, y_l, y_h, z_l, z_h, cloudMap_);
        for (auto &c : gate_list)  update_moving_shape(c, dt, x_l, x_h, y_l, y_h, z_l, z_h, cloudMap_);

        // Update grid obstacle map
        pcl2grid();
    }
  };

}

#endif
