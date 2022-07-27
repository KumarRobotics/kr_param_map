#ifndef _UNI_MP_HPP
#define _UNI_MP_HPP

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map_utils/map_basics.hpp>

#include <map_utils/grid_map.hpp>
#include <map_utils/geo_map.hpp>


namespace param_mpl
{
   
  struct PlanParams
  {
    //box limits
    double max_vel_, max_acc_, max_jerk_, max_snap_;
    int ctrl_order_, ctrl_res_;
    double durt_;
  };


  class UniPlanner
  {
  private:

    /* map utils */
    param_env::GridMap::Ptr grid_map_;
    param_env::MapParams mpa_;
    param_env::GeoMap::Ptr geo_map_;


    /* planning utils*/
    param_mpl::PlanParams ppa_;
    std::vector<Eigen::Vector3d> control_inputs_;

  public:

    UniPlanner() = default;

    ~UniPlanner() {}

    void initGridMap(param_env::GridMap::Ptr &grid_map)
    {
      grid_map_ = grid_map;
      grid_map->getMapParams(mpa_);
    }

    void initGeoMap(param_env::GeoMap::Ptr &geo_map)
    {
      geo_map_ = geo_map;
    }

    
    void initPlanner(param_mpl::PlanParams &ppa)
    {
      ppa_ = ppa;

      // set up the control input set
      double u = ppa_.u_max;
      double du = u * ppa_.ctrl_res_;

      for (double dx = -u; dx <= u; dx += du)
        for (double dy = -u; dy <= u; dy += du)
          for (double dz = -u; dz <= u; dz += du)
          {
            control_inputs_.push_back(Eigen::Vector3d(dx, dy, dz));
          }
            
    }




  };

}

#endif