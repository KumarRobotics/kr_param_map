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
#include <plan_env/Obsinfo.h>

namespace param_env {

  struct MapGenParams
  {
    /* parameters for map generator */
    double static_cylinder_ratio_, static_circle_ratio_, static_gate_ratio_, static_ellip_ratio_, static_poly_ratio_, static_ball_ratio_;
    double dyn_cylinder_ratio_, dyn_circle_ratio_, dyn_gate_ratio_, dyn_ellip_ratio_, dyn_poly_ratio_, dyn_ball_ratio_;
    double w1_, w2_, w3_;
    bool add_noise_ = false;
  };
  
  class StructMapGenerator
  {
  private:

    pcl::PointCloud<pcl::PointXYZ> cloudMap_, moving_cloudMap_; // moving_cloudMap_ only for visualization, while cloudmap only contain static obs that's sent to grid map

    param_env::GridMap grid_map_;
    param_env::GeoMap geo_map_;

    param_env::GridMapParams mpa_;
    param_env::MapGenParams mgpa_;

    default_random_engine eng;
    uniform_real_distribution<double> rand_theta, rand_w, rand_h, rand_cw, rand_radiu, rand_vx, rand_vy, rand_vz;

  public:

    // dyn param
    std::vector<param_env::Cylinder> cyl_list; std::vector<param_env::CircleGate>  cir_list;
    std::vector<param_env::RectGate>  gate_list; std::vector<param_env::Ellipsoid>  ellip_list;
    std::vector<param_env::Polyhedron> poly_list; std::vector<param_env::Sphere> ball_list;

    Eigen::Vector3d vel, vel_l, vel_h, vel_bound, acc_bound;

    StructMapGenerator() = default;
    ~StructMapGenerator() {}

    void getGridMap(param_env::GridMap& grid_map)
    {
      grid_map = grid_map_;
    }

    template<class T>
    int updatePts(T &geo_rep, bool if_dyn = false)
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
            if (if_dyn)
            {
              moving_cloudMap_.points.push_back(pt_random);
            }
            else
            {
              cloudMap_.points.push_back(pt_random);
            }

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
            if (if_dyn)
            {
              moving_cloudMap_.points.push_back(pt_random);
            }
            else
            {
              cloudMap_.points.push_back(pt_random);
            }
            geo_rep.cloud->push_back(pt_random); // get cloud at 1st iter
          }
        }

      }
      return cur_grids;
    }

    template<class T>
    int updatePtsLight(T &geo_rep, bool if_dyn = false)
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
              
              if (if_dyn)
              {
                moving_cloudMap_.points.push_back(pt_random);
              }
              else
              {
                cloudMap_.points.push_back(pt_random);
              }
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
            if (if_dyn)
            {
              moving_cloudMap_.points.push_back(pt_random);
            }
            else
            {
              cloudMap_.points.push_back(pt_random);
            }
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

      void AddGround(){
          float min_x = mpa_.basic_mp_.min_range_(0);
          float min_y = mpa_.basic_mp_.min_range_(1);
          float min_z = mpa_.basic_mp_.min_range_(2);
          float max_x = mpa_.basic_mp_.max_range_(0);
          float max_y = mpa_.basic_mp_.max_range_(1);
          float max_z = mpa_.basic_mp_.max_range_(2);
          for(float x = min_x + 1e-3; x <= max_x - 1e-3; x += 0.2){
              for(float y = min_y + 1e-3; y <= max_y - 1e-3; y += 0.2){
                  pcl::PointXYZ pt;
                  pt.x = x;
                  pt.y = y;
                  pt.z = 1e-3;
                  cloudMap_.points.push_back(pt);
              }
          }
      }


    void getPC(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {
      cloudMap_.width = cloudMap_.points.size();
      cloudMap_.height = 1;
      cloudMap_.is_dense = true;

      cloudMap = cloudMap_;
    }

    void getMovingPC(pcl::PointCloud<pcl::PointXYZ> &moving_cloudMap)
    {
      moving_cloudMap_.width = moving_cloudMap_.points.size();
      moving_cloudMap_.height = 1;
      moving_cloudMap_.is_dense = true;

      moving_cloudMap = moving_cloudMap_;
    }

    void clear(){
      cloudMap_.clear();
      moving_cloudMap_.clear();
      grid_map_.clearAllOcc();
      geo_map_.clearAll();
    }

    //it should be called after the random map gene
    void resetMap()
    {

      cloudMap_.clear();
      moving_cloudMap_.clear();

      std::vector<param_env::Polyhedron> polyhedron;
      std::vector<param_env::Cylinder> cylinder;
      std::vector<param_env::Ellipsoid> ellipsoid;
      std::vector<param_env::CircleGate> circle_gate;
      std::vector<param_env::RectGate> rect_gate;
      std::vector<param_env::Sphere> ball;

      geo_map_.getPolyhedron(polyhedron);
      geo_map_.getCylinder(cylinder);
      geo_map_.getEllipsoid(ellipsoid);
      geo_map_.getCircleGate(circle_gate);
      geo_map_.getRectGate(rect_gate);
      geo_map_.getSphere(ball);

      traversePts(polyhedron);
      traversePts(cylinder);
      traversePts(ellipsoid);
      traversePts(circle_gate);
      traversePts(rect_gate);
      traversePts(ball);
    }

    // Just for randomly static generation case:
    void change_ratios(double &seed,float dt)
    {
      eng.seed(seed);

      mgpa_.static_cylinder_ratio_ = mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.static_circle_ratio_   = 0.1  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.static_gate_ratio_     = 0.1  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.static_ellip_ratio_    = 0.5  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);
      mgpa_.static_poly_ratio_     = 0.5  * mgpa_.w2_ * rand_w(eng) * rand_w(eng);

      generate(false); // false
    }

    void randomUniMapGen(param_env::MapGenParams &mgpa, double &seed, bool if_dyn)
    {
      mgpa_ = mgpa;
      rand_w     = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w2_);
      rand_radiu = uniform_real_distribution<double>(mgpa_.w1_, mgpa_.w3_);
      rand_h     = uniform_real_distribution<double>(0.8 * mpa_.basic_mp_.map_size_(2), mpa_.basic_mp_.map_size_(2));
      eng.seed(seed);
      generate(if_dyn);
    }

    void generate(bool dyn_mode)
    {
      int cylinder_grids, circle_grids, gate_grids, ellip_grids, poly_grids, sphere_grids;
      // rand vel for each obs
      if(dyn_mode){
        rand_vx = uniform_real_distribution<double>(-vel_h(0), vel_h(0));
        rand_vy = uniform_real_distribution<double>(-vel_h(1), vel_h(1));
        rand_vz = uniform_real_distribution<double>(-vel_h(2), vel_h(2));
      }

      grid_map_.setUniRand(eng);

      int all_grids = ceil(mpa_.basic_mp_.map_volume_ / std::pow(mpa_.resolution_, 3));
      if(!dyn_mode) {
          cylinder_grids = ceil(all_grids * mgpa_.static_cylinder_ratio_);
          circle_grids = ceil(all_grids * mgpa_.static_circle_ratio_);
          gate_grids = ceil(all_grids * mgpa_.static_gate_ratio_);
          ellip_grids = ceil(all_grids * mgpa_.static_ellip_ratio_);
          poly_grids = ceil(all_grids * mgpa_.static_poly_ratio_);
          sphere_grids = ceil(all_grids * mgpa_.static_ball_ratio_);
      }else{
          cylinder_grids = ceil(all_grids * mgpa_.dyn_cylinder_ratio_);
          circle_grids = ceil(all_grids * mgpa_.dyn_circle_ratio_);
          gate_grids = ceil(all_grids * mgpa_.dyn_gate_ratio_);
          ellip_grids = ceil(all_grids * mgpa_.dyn_ellip_ratio_);
          poly_grids = ceil(all_grids * mgpa_.dyn_poly_ratio_);
          sphere_grids = ceil(all_grids * mgpa_.dyn_ball_ratio_);
      }
      Eigen::Vector3d bound;
      Eigen::Vector3d cpt, goal; // center point, goal point

      // generate cylinders
      int cur_grids = 0;
      double w, h;
      int count = 0;
      while (cur_grids < cylinder_grids)
      {
        grid_map_.getUniRandPos(cpt);
        grid_map_.getUniRandPos(goal);

        h = rand_h(eng);
        w = 0.1 + rand_w(eng);
        param_env::Cylinder cylinder(cpt, w, h);

        Eigen::Vector3d v_zero = Eigen::Vector3d::Zero(3,1);
        Eigen::Vector3d a_zero = Eigen::Vector3d::Zero(3,1);
        cylinder.setVel(v_zero);
        cylinder.setAcc(a_zero);
        cylinder.setGoal(goal);
        cur_grids += updatePts(cylinder, dyn_mode); // update pcl
        geo_map_.add(cylinder);

        if(dyn_mode){
          cyl_list.push_back(cylinder);  // add dyn_obs_list
        }
      }
      cylinder_grids = cur_grids;

      cur_grids = 0;

      // sphere, can be dynamic obstacle
      double r;
      while (cur_grids < sphere_grids)
      {
          grid_map_.getUniRandPos(cpt);
          grid_map_.getUniRandPos(goal);
          goal(2) = cpt(2) + 0.1;
          r = rand_radiu(eng);
          bound << r, r, r;
          param_env::Sphere ball(cpt, r);

          Eigen::Vector3d v_zero = Eigen::Vector3d::Zero(3,1);
          Eigen::Vector3d a_zero = Eigen::Vector3d::Zero(3,1);
          ball.init(cpt, bound , r);
          ball.setVel(v_zero);
          ball.setAcc(a_zero);
          ball.setGoal(goal);
          cur_grids += updatePts(ball, dyn_mode); // update pcl
          geo_map_.add(ball);

          if(dyn_mode){
              ball_list.push_back(ball);  // add dyn_obs_list
          }
        }
        sphere_grids = cur_grids;

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
        grid_map_.getUniRandPos(goal);

        cpt(2) = 1.5;
        goal(2) = cpt(2);
        Eigen::Vector3d euler_angle;
        if(dyn_mode){
            euler_angle << 0, 0, 0;
        }else{
            euler_angle << rand_theta(eng), rand_theta(eng), rand_theta(eng);
        }
        // make it high
        bound << rand_radiu(eng), rand_radiu(eng), rand_radiu(eng) + 2;
        param_env::Ellipsoid ellip;
        ellip.init(cpt, bound, euler_angle);

        Eigen::Vector3d v_zero = Eigen::Vector3d::Zero(3,1);
        Eigen::Vector3d a_zero = Eigen::Vector3d::Zero(3,1);
        ellip.setVel(v_zero);
        ellip.setAcc(a_zero);
        ellip.setGoal(goal);

        cur_grids += updatePts(ellip, dyn_mode);
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
      std::cout << "+++ spheres    : " << 100 * float(sphere_grids) / float(all_grids)   << "%           +++" << std::endl;
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


    /* Get polynomial traj for each obstacle(ellipsoid for now) */
    void GetObsPolyTraj(){
        Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(3,2); // consider only start and goal point
        Eigen::Vector3d start_vel, start_acc, end_vel, end_acc, cpt ,goal;

        double max_vel_ = vel_bound.norm(), max_acc_ = acc_bound.norm();
        double duration;
        for(auto & c : ellip_list){
            // get start pos, vel, acc
            c.getCenter(cpt);
            c.getVel(start_vel);
            c.getAcc(start_acc);
            c.getGoal(goal);

            // get goal pos vel acc
            end_vel << 0,0,0;
            end_acc << 0,0,0;

            double dist = (cpt - goal).norm();
            duration = pow(max_vel_, 2) / max_acc_ > dist ?
                       sqrt(dist / max_acc_)  : (dist - pow(max_vel_, 2) / max_acc_) /max_vel_ + 2 * max_vel_ / max_acc_;

            PolynomialTraj obs_traj = PolynomialTraj::one_segment_traj_gen_3_order(cpt, start_vel, start_acc, goal, end_vel, end_acc, duration);
            c.setTraj(obs_traj);
            c.traj_start_time = ros::Time::now().toSec();
            c.traj_duration = duration;
            c.obs_init_time = ros::Time::now().toSec();

            PolynomialTraj back_obs_traj = PolynomialTraj::one_segment_traj_gen_3_order(goal, start_vel, start_acc, cpt, end_vel, end_acc, duration);
            c.setBackTraj(back_obs_traj);
            c.back_traj_start_time = c.traj_start_time + c.traj_duration;
            c.back_traj_duration = duration;
        }
    }

    /* get pts on obs's traj w.r.t. world time, this is just for rviz visualization */
    void dyn_generate_traj(double t) {
        moving_cloudMap_.clear();
        Eigen::Vector3d cur_cpt, next_cpt, dist;
        PolynomialTraj poly;
        double t_cur;

        for (auto &c: ellip_list) {
            c.getCenter(cur_cpt);
            //cout << cur_cpt.transpose() << endl;
            if(t > c.back_traj_start_time + c.back_traj_duration){
                c.traj_start_time += c.traj_duration + c.back_traj_duration;
                c.back_traj_start_time = c.traj_start_time + c.traj_duration;
            }

            if(t <= c.traj_duration + c.traj_start_time){
                c.getTraj(poly);
                t_cur = t - c.traj_start_time;
            }
            else if(t <= c.back_traj_start_time + c.back_traj_duration){
                c.getBackTraj(poly);
                t_cur = t - c.back_traj_start_time;
            }

            next_cpt = poly.evaluate(t_cur);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
            dist = next_cpt - cur_cpt;
            move_clouds(c, cloud_obs, dist);
            *(c.cloud) = *cloud_obs;
            moving_cloudMap_ += *cloud_obs;
            c.setCenter(next_cpt);
        }
    }


      // just for ellip obs for now
      void SendObsTrajInfo(plan_env::Obsinfo &info, double horizons, double time_now) {
          info.heights.clear();
          info.radiuss.clear();
          info.s0.clear();
          info.s1.clear();
          info.s2.clear();

          info.send_msg_time.clear();
          info.coeff.clear();
          info.horizons.clear();
          info.trajs_start_time.clear();
          info.durations.clear();

          vector<vector<double>> cof;
          vector<double> ts;
          vector<int> which_poly;
          PolynomialTraj poly;


          info.header.stamp = ros::Time::now();

          info.shape = "ellips";
          info.layouts[0] = ellip_list.size();   // number of obs
          info.layouts[1] = 4;  // n polys, 4 coeff, 3 dim
          info.layouts[2] = 3;  // n polys, 4 coeff, 3 dim
          Eigen::Vector3d pt;
          Eigen::Matrix3d S;

          for (int i = 0; i < ellip_list.size(); i++) {
              which_poly.clear();
              info.horizons.push_back(horizons);
              info.send_msg_time.push_back(time_now); // current world time
              S = ellip_list[i].getS();
              info.s0.push_back(S(0,0));
              info.s1.push_back(S(1,1));
              info.s2.push_back(S(2,2));

              // find out which polys currently in and will end
              double t_in_period = std::fmod(time_now - ellip_list[i].obs_init_time, ellip_list[i].traj_duration + ellip_list[i].back_traj_duration);
              int bina = 0;
              double t_start = 0.0;
              if (t_in_period <= ellip_list[i].traj_duration) {
                  t_start = ellip_list[i].traj_start_time;
                  bina = 0;
              } else {
                  t_start = ellip_list[i].back_traj_start_time;
                  bina = 1;
              }
              while (t_start <= horizons + time_now) {
                  which_poly.push_back(bina);
                  info.trajs_start_time.push_back(t_start);
                  if (bina == 0) {
                      t_start += ellip_list[i].traj_duration;
                      info.durations.push_back(ellip_list[i].traj_duration);
                  }
                  else {
                      t_start += ellip_list[i].back_traj_duration;
                      info.durations.push_back(ellip_list[i].back_traj_duration);
                  }
                  bina = (bina == 0) ? 1 : 0;
              }
              info.poly_nums_each_obs.push_back(which_poly.size());

              for (int dim = 0; dim <= 2; dim++) {
                  for(int k = 0; k < which_poly.size(); k++){
                      if(which_poly[k] == 0){
                          ellip_list[i].getTraj(poly);
                      }else{
                          ellip_list[i].getBackTraj(poly);
                      }
                      cof = poly.getCoef(dim);
                      for (int seg = 0; seg < cof.size(); seg++) {
                          for (int j = 0; j < cof[seg].size(); j++) {
                              info.coeff.push_back(cof[seg][j]);
                          }
                      }
                  }
              }
         }
     }
  };
}

#endif
