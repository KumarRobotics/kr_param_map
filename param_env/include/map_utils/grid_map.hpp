#ifndef _GRID_MAP_HPP
#define _GRID_MAP_HPP

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <map_utils/map_basics.hpp>


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
    Eigen::Vector3i map_min_idx_, map_max_idx_;
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
      //std::cout << "buffer_size " << buffer_size << std::endl;
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

    // fill occupancies with point clouds
    void fillMap(pcl::PointCloud<pcl::PointXYZ> &cloudMap, double &inflated)
    {
      Eigen::Vector3d ob_pt;

      if (inflated != 0.0){
        double step = inflated / mp_.resolution_;

        // std::cout << "step is " << step << std::endl;
        // std::cout << "inflated is " << inflated << std::endl;
         

        for (auto pt : cloudMap)
        {
          ob_pt << pt.x, pt.y, pt.z;

          for (double x = -inflated; x <= inflated; x+=mp_.resolution_)
            for (double y = -inflated; y <= inflated; y+=mp_.resolution_)
              for (double z = -inflated; z <= inflated; z+=mp_.resolution_) {
                setOcc(Eigen::Vector3d(ob_pt(0) + x, ob_pt(1) + y, ob_pt(2) + z));
              }
        }

      }else{

        for (auto pt : cloudMap)
        {
          ob_pt << pt.x, pt.y, pt.z;
          setOcc(ob_pt);
        }

      }

    }


    void publishMap(pcl::PointCloud<pcl::PointXYZ> &cloudMap)
    {

      pcl::PointXYZ pt;
      cloudMap.clear();

      Eigen::Vector3d min_pos = mp_.basic_mp_.min_range_;
      Eigen::Vector3d max_pos = mp_.basic_mp_.max_range_;

      Eigen::Vector3i min_id, max_id;

      posToIndex(min_pos, min_id);

      double temp = mp_.resolution_ / 2;
      posToIndex(max_pos - Eigen::Vector3d(temp, temp, temp), max_id);

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z)
          {
            if (occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] > mp_.min_thrd_) 
            {
              //std::cout << occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] << std::endl;
              Eigen::Vector3d pos;
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
