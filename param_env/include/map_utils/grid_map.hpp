#ifndef _GRID_MAP_HPP
#define _GRID_MAP_HPP

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <map_utils/map_basics.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace param_env
{
  using namespace std;

  struct GridMapParams
  {

    BasicMapParams basic_mp_;

    /* grid map adjusted parameters */
    double resolution_, global_density_;

    /* deducted paramaters */
    Eigen::Vector3i map_grid_size_;         // map range in index
    int map_grid_size_ytz_;
    double inv_resolution_;

    /*advanced parameters*/
    double clamp_min_log_ = 0.01;
    double clamp_max_log_ = 0.99;
    double min_thrd_ = 0.80;
  };

  class GridMap
  {
  private:
    std::vector<double> occupancy_buffer_; // 0 is free, 1 is occupied

    GridMapParams mp_;

    //get random position in the map
    uniform_real_distribution<double> rand_x_;
    uniform_real_distribution<double> rand_y_;
    uniform_real_distribution<double> rand_z_;
    default_random_engine eng_;

    std::vector<Eigen::Vector3d> obs_pts; // for further processing (obstacles avoidance)

  public:
    GridMap() = default;

    ~GridMap() {}

    void initMap(const GridMapParams &mpa)
    {

      mp_ = mpa;

      // update grid map parameters
      mp_.inv_resolution_ = 1.0 / mp_.resolution_;
      for (int i = 0; i < 3; ++i)
      {
        mp_.map_grid_size_(i) = ceil(mp_.basic_mp_.map_size_(i) / mp_.resolution_);
      }

      mp_.map_grid_size_ytz_ = mp_.map_grid_size_(1) * mp_.map_grid_size_(2);

      int buffer_size = mp_.map_grid_size_(0) * mp_.map_grid_size_ytz_;
      //std::cout << "mp_.map_grid_size_ " << mp_.map_grid_size_ << std::endl;
      occupancy_buffer_.resize(buffer_size);
      fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), mp_.clamp_min_log_);

      printMapInfo();
    }

    void printMapInfo()
    {

      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;
      std::cout << "+++++++Grid Map Information+++++++++++" << std::endl;
      std::cout << "+++ resolution : " << mp_.resolution_  << std::endl;
      std::cout << "+++ map volume : " << mp_.basic_mp_.map_volume_  << std::endl;
      std::cout << "+++ origin     : " << mp_.basic_mp_.map_origin_(0) << " " << mp_.basic_mp_.map_origin_(1) << " " << mp_.basic_mp_.map_origin_(2) << std::endl;
      std::cout << "+++ size       : " << mp_.basic_mp_.map_size_(0) << " " << mp_.basic_mp_.map_size_(1) << " " << mp_.basic_mp_.map_size_(2) << std::endl;
      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;

    }

    Eigen::Vector3d evaluateEnv(double mav_radius)
    {
      int occ_num = 0, con_num = 0;
      float max_dis = 0.0; //SquaredDistance
      
      std::vector<Eigen::Vector3i> neighbors;
      neighbors.push_back(Eigen::Vector3i(0, 0,  1));
      neighbors.push_back(Eigen::Vector3i(0, 0, -1));
      neighbors.push_back(Eigen::Vector3i(0,  1, 0));
      neighbors.push_back(Eigen::Vector3i(0, -1, 0));
      neighbors.push_back(Eigen::Vector3i( 1, 0, 0));
      neighbors.push_back(Eigen::Vector3i(-1, 0, 0));

      // kd tree setup
      pcl::PointCloud<pcl::PointXYZ> cloud_all_map;
      getObsPts(cloud_all_map);

      if (cloud_all_map.size()<= 0)
      {
        std::cout << "no cloud for evaluation" << std::endl;

        return Eigen::Vector3d(0, 0, 0);
      } 
      pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
      kdtreeMap.setInputCloud(cloud_all_map.makeShared());
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      for (int x = 0; x < mp_.map_grid_size_(0); ++x)
        for (int y = 0; y < mp_.map_grid_size_(1); ++y)
          for (int z = 0; z < mp_.map_grid_size_(2); ++z)
          {
            if (occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] > mp_.min_thrd_) 
            {
              //std::cout << occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] << std::endl;
              occ_num += 1;


              // check surroundings
              for (auto &nbr : neighbors)
              {

                Eigen::Vector3i pt = Eigen::Vector3i(x, y, z) + nbr;
                if (!isOcc(pt))
                {
                  con_num += 1;
                  break;
                }
              }

            }else
            {
              //free grid, let's compute the dispersion
              pointIdxRadiusSearch.clear();
              pointRadiusSquaredDistance.clear();


              Eigen::Vector3d pos;
              indexToPos(Eigen::Vector3i(x, y, z), pos);

              pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));


              if (kdtreeMap.nearestKSearch(searchPoint, 1,
                                            pointIdxRadiusSearch,
                                            pointRadiusSquaredDistance) > 0)
              {
                if (pointRadiusSquaredDistance[0] > max_dis)
                {
                  max_dis = pointRadiusSquaredDistance[0];
                }

              }
            }
              
          }

      std::cout << "+++  std::sqrt(max_dis) : " << std::sqrt(max_dis) << std::endl;
      std::cout << "+++  occ_num : " << occ_num << std::endl;
      std::cout << "+++  con_num  : " <<  con_num  << std::endl;
      double density_index   = ((double) occ_num) / ((double)(mp_.map_grid_size_(0) * mp_.map_grid_size_(1) * mp_.map_grid_size_(2)));
      double structure_index = ((double) con_num) / (std::max(0.1, (double)(occ_num)));

      double clutter_index   = mav_radius / std::sqrt(max_dis);


      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;
      std::cout << "+++++++Enviornment Complexity Index (ECI) +++++++++++" << std::endl;
      std::cout << "+++ density index     : " << density_index  << std::endl;
      std::cout << "+++ clutter_index     : " << clutter_index  << std::endl;
      std::cout << "+++ structure index   : " << structure_index  << std::endl;
      std::cout << "++++++++++++++++++++++++++++++++++++++" << std::endl;

      return Eigen::Vector3d(density_index, clutter_index, structure_index);
    }


    void ECIgenerate(double density_index, double clutter_index,  double structure_index)
    {


      double cur_density_index = 0.0;
      double cur_clutter_index = 0.0;
      double cur_structure_index = 0.0;

      int occ_num = 0, con_num = 0;
      float max_dis = 0.0; //SquaredDistance

      int exp_obs_num = int(density_index * mp_.map_grid_size_(0) * mp_.map_grid_size_(1) * mp_.map_grid_size_(2));
      int sp_num = int((structure_index) * exp_obs_num);
      int ave_sample_num =  int( exp_obs_num * 1.0 / (sp_num  * 1.0 ));

      std::cout <<  "exp_obs_num is " << exp_obs_num << std::endl;
      std::cout <<  "ave_sample_num is " << ave_sample_num << std::endl;
      std::cout <<  "sp_num is " << sp_num << std::endl;


      std::uniform_real_distribution<double> rand_scale_1 = std::uniform_real_distribution<double>(0.0, mp_.map_grid_size_(0)); 
      std::uniform_real_distribution<double> rand_scale_2 = std::uniform_real_distribution<double>(0.0, mp_.map_grid_size_(1));
      std::uniform_real_distribution<double> rand_scale_3 = std::uniform_real_distribution<double>(0.0, mp_.map_grid_size_(2));


      std::normal_distribution<double> rand_x = std::normal_distribution<double>(0, clutter_index); 
      std::normal_distribution<double> rand_y = std::normal_distribution<double>(0, clutter_index); 
      std::normal_distribution<double> rand_z = std::normal_distribution<double>(0, clutter_index); 
      // Mean and Standard deviation

      Eigen::Vector3d cpt, ob_pt;  // center points, object points

      while (abs(cur_density_index- density_index) > 1e-3) {

        getUniRandPos(cpt);  //uniform random points

        
        if (isOcc(cpt) != 0) {
          continue;
        }
        setOcc(cpt);
        occ_num += 1;


        int i = 0;
        while (i < sp_num)
        {
          Eigen::Vector3d d_step(rand_scale_1(eng_) * rand_x(eng_), rand_scale_2(eng_) * rand_y(eng_), rand_scale_3(eng_) * rand_z(eng_));
          //std::cout <<  "d_step is " << d_step << std::endl;
          ob_pt = cpt + mp_.resolution_ * d_step;
          i += 1;

          if (isOcc(ob_pt) != 0) {
            continue;
          }



          setOcc(ob_pt);
          occ_num += 1;
        }        
        cur_density_index  = ((double) occ_num) / ((double)(mp_.map_grid_size_(0) * mp_.map_grid_size_(1) * mp_.map_grid_size_(2)));

        
      }

      return;
    }


    double getMapMaxDis()
    {
      // kd tree setup
      pcl::PointCloud<pcl::PointXYZ> cloud_all_map;
      getObsPts(cloud_all_map);

      double max_dis;

      if (cloud_all_map.size()<= 0)
      {
        std::cout << "no cloud for evaluation" << std::endl;

        return 0.0;
      } 
      pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
      kdtreeMap.setInputCloud(cloud_all_map.makeShared());
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      for (int x = 0; x < mp_.map_grid_size_(0); ++x)
        for (int y = 0; y < mp_.map_grid_size_(1); ++y)
          for (int z = 0; z < mp_.map_grid_size_(2); ++z)
          {
            if (occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] <= mp_.min_thrd_) 
            {
              //free grid, let's compute the dispersion
              pointIdxRadiusSearch.clear();
              pointRadiusSquaredDistance.clear();

              Eigen::Vector3d pos;
              indexToPos(Eigen::Vector3i(x, y, z), pos);

              pcl::PointXYZ searchPoint(pos(0), pos(1), pos(2));


              if (kdtreeMap.nearestKSearch(searchPoint, 1,
                                            pointIdxRadiusSearch,
                                            pointRadiusSquaredDistance) > 0)
              {
                if (pointRadiusSquaredDistance[0] > max_dis)
                {
                  max_dis = pointRadiusSquaredDistance[0];
                }

              }
            }
              
          }

      return std::sqrt(max_dis);
    }   


    void addGround(double h) //ground height
    {

      int h_grids =  ceil(h / mp_.resolution_);
      for (int x = 0; x < mp_.map_grid_size_(0); ++x)
        for (int y = 0; y < mp_.map_grid_size_(1); ++y)
          for (int z = 0; z < h_grids; ++z)
          {

            Eigen::Vector3i idx(x, y, z);
            if (isOcc(idx) != 0) {
              continue;
            }
            setOcc(idx);
          }

    }

    // get the map parameters
    void getMapParams(GridMapParams &mpa)
    {
      mpa = mp_;
    }

    // set random seed for the map
    void setUniRand(default_random_engine &eng){

      rand_x_ = uniform_real_distribution<double>(mp_.basic_mp_.min_range_(0), mp_.basic_mp_.max_range_(0));
      rand_y_ = uniform_real_distribution<double>(mp_.basic_mp_.min_range_(1), mp_.basic_mp_.max_range_(1));
      rand_z_ = uniform_real_distribution<double>(mp_.basic_mp_.min_range_(2), mp_.basic_mp_.max_range_(2));

      eng_ = eng;

    }

    // reset map buffers
    void resetBuffer(Eigen::Vector3d min_pos,
                     Eigen::Vector3d max_pos)
    {
      min_pos(0) = max(min_pos(0), mp_.basic_mp_.min_range_(0));
      min_pos(1) = max(min_pos(1), mp_.basic_mp_.min_range_(1));
      min_pos(2) = max(min_pos(2), mp_.basic_mp_.min_range_(2));

      max_pos(0) = min(max_pos(0), mp_.basic_mp_.max_range_(0));
      max_pos(1) = min(max_pos(1), mp_.basic_mp_.max_range_(1));
      max_pos(2) = min(max_pos(2), mp_.basic_mp_.max_range_(2));

      Eigen::Vector3i min_id, max_id;

      posToIndex(min_pos, min_id);

      double temp = mp_.resolution_ / 2;
      posToIndex(max_pos - Eigen::Vector3d(temp, temp, temp), max_id);

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z)
          {
            occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] = mp_.clamp_min_log_;
          }
    }

    // set occupancy to the map
    void setOcc(const Eigen::Vector3d &pos)
    {
      Eigen::Vector3i id;
      posToIndex(pos, id);
      if (!isInMap(id))
        return;

      occupancy_buffer_[getBufferCnt(id)] = mp_.clamp_max_log_;
      obs_pts.push_back(pos);
    }

    // set occupancy to the map
    void setOcc(const Eigen::Vector3i &id)
    {
      Eigen::Vector3d pos;
      indexToPos(id, pos);
      if (!isInMap(id))
        return;

      occupancy_buffer_[getBufferCnt(id)] = mp_.clamp_max_log_;
      obs_pts.push_back(pos);
    }


    // fill occupancies with point clouds
    void fillMap(pcl::PointCloud<pcl::PointXYZ> &cloudMap, double inflated)
    {
      Eigen::Vector3d ob_pt, infla_pt;

      if (inflated > 0.0){
        int step = std::ceil(inflated / mp_.resolution_);

        for (auto &pt : cloudMap)
        {
          ob_pt << pt.x, pt.y, pt.z;

          for (int step_x = -step; step_x <= step; step_x++)
          {
            for (int step_y = -step; step_y <= step; step_y++)
            {
              for (int step_z = -step; step_z <= step; step_z++)
              {
                infla_pt << ob_pt(0) + step_x * mp_.resolution_, 
                            ob_pt(1) + step_y * mp_.resolution_, 
                            ob_pt(2) + step_z * mp_.resolution_;

                if (!isOcc(infla_pt))
                {
                  setOcc(infla_pt);
                }

              }
            }
          }
        }

      }else{

        for (auto &pt : cloudMap)
        {
          ob_pt << pt.x, pt.y, pt.z;
          if (!isOcc(ob_pt))
          {
            setOcc(ob_pt);
          }
        }

      }

    }

    void getObsPts(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {
      pcl::PointXYZ pt;
      cloudMap.clear();
      for (auto &pos : obs_pts)
      {
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloudMap.push_back(pt);
      }

      cloudMap.width = cloudMap.points.size();
      cloudMap.height = 1;
      cloudMap.is_dense = true;
      return;


    }


    void publishMap(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      pcl::PointXYZ pt;
      cloudMap.clear();

      Eigen::Vector3d max_pos = mp_.basic_mp_.max_range_;
      Eigen::Vector3d pos;

      for (int x = 0; x < mp_.map_grid_size_(0); ++x)
        for (int y = 0; y < mp_.map_grid_size_(1); ++y)
          for (int z = 0; z < mp_.map_grid_size_(2); ++z)
          {
            if (occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] > mp_.min_thrd_) 
            {
              //std::cout << occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] << std::endl;
              indexToPos(Eigen::Vector3i(x, y, z), pos);

              pt.x = pos(0);
              pt.y = pos(1);
              pt.z = pos(2);
              cloudMap.push_back(pt);
            }

          }


      cloudMap.width = cloudMap.points.size();
      cloudMap.height = 1;
      cloudMap.is_dense = true;

    }

    void publish2dMap(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      pcl::PointXYZ pt;
      cloudMap.clear();

      Eigen::Vector3d max_pos = mp_.basic_mp_.max_range_;
      Eigen::Vector3d pos;


      
      for (int x = 0; x < mp_.map_grid_size_(0); ++x)
      {
        for (int y = 0; y < mp_.map_grid_size_(1); ++y)
        {
          bool occ = false;
          for (int z = 0; z < mp_.map_grid_size_(2); ++z)
          {
            
            if (occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] > mp_.min_thrd_) 
            {
              //std::cout << occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] << std::endl;
              occ = true;
              indexToPos(Eigen::Vector3i(x, y, z), pos);
              break;
            }
          }
          if (occ)
          {
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = 0.0;
            cloudMap.push_back(pt);
          }

        }
      }

      cloudMap.width = cloudMap.points.size();
      cloudMap.height = 1;
      cloudMap.is_dense = true;



    }


    // clear all the obs
    void clearAllOcc()
    {
      fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), mp_.clamp_min_log_);
      obs_pts.clear();
      
    }


    // check if the pos is occupied
    int isOcc(const Eigen::Vector3d &pos)
    {
      Eigen::Vector3i id;
      posToIndex(pos, id);
      if (!isInMap(id)){
        return -1;
      }
        
      // (x, y, z) -> x*ny*nz + y*nz + z
      return occupancy_buffer_[getBufferCnt(id)] > mp_.min_thrd_ ? 1 : 0;
    }

    // check if the pos is occupied
    int isOcc(const Eigen::Vector3i &id)
    {
      if (!isInMap(id)){
        return -1;
      }
        
      // (x, y, z) -> x*ny*nz + y*nz + z
      return occupancy_buffer_[getBufferCnt(id)] > mp_.min_thrd_ ? 1 : 0;
    }


    // check if the pos is in map range
    bool isInMap(const Eigen::Vector3d &pos)
    {

      if (pos(0) < mp_.basic_mp_.min_range_(0) || pos(0) >= mp_.basic_mp_.max_range_(0) ||
          pos(1) < mp_.basic_mp_.min_range_(1) || pos(1) >= mp_.basic_mp_.max_range_(1) ||
          pos(2) < mp_.basic_mp_.min_range_(2) || pos(2) >= mp_.basic_mp_.max_range_(2))
      {
        return false;
      }

      return true;
    }

    // check if the pos index is in map range
    bool isInMap(const Eigen::Vector3i &id)
    {
      Eigen::Vector3d pos;
      indexToPos(id, pos);
      return isInMap(pos);
    }

    double getGridVal(double p)
    {

      return floor(p * mp_.inv_resolution_) * mp_.resolution_ + mp_.resolution_ / 2.0;
    }


    void posToIndex(const Eigen::Vector3d &pos,
                    Eigen::Vector3i &id)
    {
      for (int i = 0; i < 3; ++i)
      {
        id(i) = floor((pos(i) - mp_.basic_mp_.map_origin_(i)) * mp_.inv_resolution_);
      }
    }

    void indexToPos(const Eigen::Vector3i &id,
                    Eigen::Vector3d &pos)
    {
      pos = mp_.basic_mp_.map_origin_;
      for (int i = 0; i < 3; ++i)
      {
        pos(i) += (id(i) + 0.5) * mp_.resolution_;
      }
    }

    // get the center of the grid
    Eigen::Vector3d getGridCenterPos(const Eigen::Vector3d pos)
    {
      Eigen::Vector3d center_pos;

      center_pos(0) = getGridVal(pos(0));
      center_pos(1) = getGridVal(pos(1));
      center_pos(2) = getGridVal(pos(2));

      return center_pos;
    }

    int getBufferCnt(const Eigen::Vector3i &id)
    {
      //x*ny*nz + y*nz + z
      return id(0) * mp_.map_grid_size_ytz_ + id(1) * mp_.map_grid_size_(2) + id(2);
    }

    int getBufferCnt(const Eigen::Vector3d &pos)
    {
      Eigen::Vector3i id;
      posToIndex(pos, id);
      return getBufferCnt(id);
    }

    
    void getUniRandPos(Eigen::Vector3d &pos)
    {

      pos(0) = rand_x_(eng_);
      pos(1) = rand_y_(eng_);
      pos(2) = rand_z_(eng_);

      pos = getGridCenterPos(pos);

    }

    typedef shared_ptr<GridMap> Ptr;
  

  };

}

#endif
