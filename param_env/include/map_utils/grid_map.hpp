#ifndef _GRID_MAP_HPP
#define _GRID_MAP_HPP

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "geo_utils/quickhull.hpp"
#include "geo_utils/geo_utils.hpp"

namespace param_env
{

  using namespace std;

  struct MapParams
  {

    /* grid map basic properties */
    Eigen::Vector3d map_origin_, map_size_;

    /* grid map adjusted parameters */
    double resolution_, global_density_;

    /* deducted paramaters */
    Eigen::Vector3d min_range_, max_range_; // map range in pos
    Eigen::Vector3i map_grid_size_;         // map range in index
    double map_grid_size_ytz_;
    Eigen::Vector3i map_min_idx_, map_max_idx_;
    double inv_resolution_;
  };

  class GridMap
  {
  private:
    std::vector<double> occupancy_buffer_; // 0 is free, 1 is occupied
    double clamp_min_log_ = 0.01;
    double clamp_max_log_ = 0.99;
    double min_thrd_ = 0.80;
    MapParams mp_;

    //get random position in the map
    uniform_real_distribution<double> rand_x_;
    uniform_real_distribution<double> rand_y_;
    uniform_real_distribution<double> rand_z_;
    default_random_engine eng_; 

  public:
    GridMap() = default;

    ~GridMap() {}

    void init(const MapParams &mpa)
    {

      mp_ = mpa;
      // update some other related paramaters
      mp_.inv_resolution_ = 1.0 / mp_.resolution_;
      mp_.min_range_ = mp_.map_origin_;
      mp_.max_range_ = mp_.map_origin_ + mp_.map_size_;

      for (int i = 0; i < 3; ++i)
      {
        mp_.map_grid_size_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
      }

      mp_.map_grid_size_ytz_ = mp_.map_grid_size_(1) * mp_.map_grid_size_(2);

      int buffer_size = mp_.map_grid_size_(0) * mp_.map_grid_size_ytz_;
      cout << "buffer size: " << buffer_size << endl;
      occupancy_buffer_.resize(buffer_size);
      fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), clamp_min_log_);


    }

    void getMapParams(MapParams &mpa)
    {

      mpa = mp_;
    }

    void resetBuffer(Eigen::Vector3d min_pos,
                     Eigen::Vector3d max_pos)
    {
      min_pos(0) = max(min_pos(0), mp_.min_range_(0));
      min_pos(1) = max(min_pos(1), mp_.min_range_(1));
      min_pos(2) = max(min_pos(2), mp_.min_range_(2));

      max_pos(0) = min(max_pos(0), mp_.max_range_(0));
      max_pos(1) = min(max_pos(1), mp_.max_range_(1));
      max_pos(2) = min(max_pos(2), mp_.max_range_(2));

      Eigen::Vector3i min_id, max_id;

      posToIndex(min_pos, min_id);

      double temp = mp_.resolution_ / 2;
      posToIndex(max_pos - Eigen::Vector3d(temp, temp, temp), max_id);

      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
          for (int z = min_id(2); z <= max_id(2); ++z)
          {
            occupancy_buffer_[getBufferCnt(Eigen::Vector3i(x, y, z))] = clamp_min_log_;
          }
    }

    void setOcc(const Eigen::Vector3d &pos)
    {
      Eigen::Vector3i id;
      posToIndex(pos, id);
      if (!isInMap(id))
        return;

      occupancy_buffer_[getBufferCnt(id)] = clamp_max_log_;
    }


    int isOcc(const Eigen::Vector3d &pos)
    {
      Eigen::Vector3i id;
      posToIndex(pos, id);
      if (!isInMap(id)){
        return -1;
      }
        
      // (x, y, z) -> x*ny*nz + y*nz + z
      return occupancy_buffer_[getBufferCnt(id)] > min_thrd_ ? 1 : 0;
    }



    void setUniRand(default_random_engine &eng){

      rand_x_ = uniform_real_distribution<double>(mp_.min_range_(0), mp_.max_range_(0));
      rand_y_ = uniform_real_distribution<double>(mp_.min_range_(1), mp_.max_range_(1));
      rand_z_ = uniform_real_distribution<double>(mp_.min_range_(2), mp_.max_range_(2));

      eng_ = eng;

    }

    
    void getUniRandPos(Eigen::Vector3d &pos)
    {

      pos(0) = rand_x_(eng_);
      pos(1) = rand_y_(eng_);
      pos(2) = rand_z_(eng_);

      pos = getGridCenterPos(pos);

    }


    bool isInMap(const Eigen::Vector3d &pos)
    {

      if (pos(0) < mp_.min_range_(0) || pos(0) >= mp_.max_range_(0) ||
          pos(1) < mp_.min_range_(1) || pos(1) >= mp_.max_range_(1) ||
          pos(2) < mp_.min_range_(2) || pos(2) >= mp_.max_range_(2))
      {
        return false;
      }

      return true;
    }

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
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.inv_resolution_);
      }
    }

    void indexToPos(const Eigen::Vector3i &id,
                    Eigen::Vector3d &pos)
    {
      pos = mp_.map_origin_;
      for (int i = 0; i < 3; ++i)
      {
        pos(i) += (id(i) + 0.5) * mp_.resolution_;
      }
    }

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
  };

}

#endif