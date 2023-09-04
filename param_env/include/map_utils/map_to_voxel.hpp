
#include <Eigen/Eigen>
#include <iostream>
#include <map_utils/grid_map.hpp>
#include <string>

#include <kr_planning_msgs/VoxelMap.h>

namespace param_env {

  inline void gridMapToVoxelMap(param_env::GridMap &grid_map,     
                       std::string &_frame_id, 
                       kr_planning_msgs::VoxelMap &map_resp)
  {
    // if (i < 10) {
    //pcl::toROSMsg(cloudMap, globalMap_pcd);
    // _all_map_cloud_pub.publish(globalMap_pcd);

    // To make sure get a consistent time in simulation
    map_resp.header.frame_id = _frame_id;
    map_resp.header.stamp = ros::Time::now();

    GridMapParams _grid_mpa;
    grid_map.getMapParams(_grid_mpa);
    std::cout << "+++ resolution : " << _grid_mpa.resolution_  << std::endl;



    map_resp.dim.x = std::ceil(_grid_mpa.basic_mp_.map_size_(0) / _grid_mpa.resolution_);
    map_resp.dim.y = std::ceil(_grid_mpa.basic_mp_.map_size_(1) / _grid_mpa.resolution_);
    map_resp.dim.z = std::ceil(_grid_mpa.basic_mp_.map_size_(2) / _grid_mpa.resolution_);
    map_resp.resolution = _grid_mpa.resolution_;
    map_resp.origin.x = _grid_mpa.basic_mp_.map_origin_(0);
    map_resp.origin.y = _grid_mpa.basic_mp_.map_origin_(1);
    map_resp.origin.z = _grid_mpa.basic_mp_.map_origin_(2);
    map_resp.data.clear();
    map_resp.data.resize(map_resp.dim.x * map_resp.dim.y * map_resp.dim.z);
    std::cout << "+++ map_resp.dim.x  : " << map_resp.dim.x  << std::endl;
    std::cout << "+++ map_resp.dim.y  : " << map_resp.dim.y  << std::endl;
    std::cout << "+++ map_resp.dim.z  : " << map_resp.dim.z  << std::endl;
    int tst = 0;

    for (int m = 0; m < map_resp.dim.z; m++) {
      for (int j = 0; j < map_resp.dim.y; j++) {
        for (int i = 0; i < map_resp.dim.x; i++) {
            
          Eigen::Vector3i idx(i, j, m);
          if(grid_map.isOcc(idx))
          {
            map_resp.data[ map_resp.dim.y *  map_resp.dim.x * m +  map_resp.dim.x * j + i ] = 100;
            tst += 1;
          }
          else
          {
            map_resp.data[ map_resp.dim.y *  map_resp.dim.x * m +  map_resp.dim.x * j + i ] = 0;
          }
          
        }
      }
    }

    std::cout << "+++ tyst  : " << tst  << std::endl;
  }


  inline void gridMapToInflaVoxelMap(param_env::GridMap &grid_map,     
                              std::string &_frame_id,
                              double infla_radius,
                              kr_planning_msgs::VoxelMap &map_resp)
  {
    // if (i < 10) {
    //pcl::toROSMsg(cloudMap, globalMap_pcd);
    // _all_map_cloud_pub.publish(globalMap_pcd);

    // To make sure get a consistent time in simulation
    map_resp.header.frame_id = _frame_id;
    map_resp.header.stamp = ros::Time::now();

    GridMapParams _grid_mpa;
    grid_map.getMapParams(_grid_mpa);


    map_resp.dim.x = std::ceil(_grid_mpa.basic_mp_.map_size_(0) / _grid_mpa.resolution_);
    map_resp.dim.y = std::ceil(_grid_mpa.basic_mp_.map_size_(1) / _grid_mpa.resolution_);
    map_resp.dim.z = std::ceil(_grid_mpa.basic_mp_.map_size_(2) / _grid_mpa.resolution_);
    map_resp.resolution = _grid_mpa.resolution_;
    map_resp.origin.x = _grid_mpa.basic_mp_.map_origin_(0);
    map_resp.origin.y = _grid_mpa.basic_mp_.map_origin_(1);
    map_resp.origin.z = _grid_mpa.basic_mp_.map_origin_(2);
    map_resp.data.clear();
    map_resp.data.resize(map_resp.dim.x * map_resp.dim.y * map_resp.dim.z);

    int step = std::ceil(infla_radius / _grid_mpa.resolution_);

    for (int m = 0; m < map_resp.dim.z; m++) {
      for (int j = 0; j < map_resp.dim.y; j++) {
        for (int i = 0; i < map_resp.dim.x; i++) {
            
          Eigen::Vector3i idx(i, j, m);
          if(grid_map.isOcc(idx))
          {
            
            for (int step_x = -step; step_x <= step; step_x++)
            {
              for (int step_y = -step; step_y <= step; step_y++)
              {
                for (int step_z = -step; step_z <= step; step_z++)
                { 
                  Eigen::Vector3i idx_inf(i + step_x, j + step_y, m + step_z);
                  if(grid_map.isInMap(idx_inf))
                  {
                    map_resp.data[ map_resp.dim.y *  map_resp.dim.x * (m + step_z) +  +  map_resp.dim.x * (j + step_y) + i + step_x] = 100;
                  }

                }
              }
            }
          }
          else
          {
            map_resp.data[ map_resp.dim.y *  map_resp.dim.x * m +  map_resp.dim.x * j + i ] = 0;
          }
        
        }
      }
    }

  }


}  // namespace param_env
