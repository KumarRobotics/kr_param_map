#ifndef _GEO_MAP_HPP
#define _GEO_MAP_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <map_utils/map_basics.hpp>
#include <string>

namespace param_env {

using namespace std;
enum GEO_TYPE {
  POLYGON = 1,
  ELLIPSE = 2,
  POLYHEDRON = 3,
  CYLINDER = 4,
  ELLIPSOID = 5,
  CIRCLEGATE = 6,
  RECTGATE = 7,
};

class GeoMap {
 private:
  // 2d set
  std::vector<Polygon> polygon_;
  std::vector<Ellipse> ellipse_;

  // 3d set
  std::vector<Polyhedron> polyhedron_;
  std::vector<Cylinder> cylinder_;
  std::vector<Ellipsoid> ellipsoid_;
  std::vector<CircleGate> circle_gate_;
  std::vector<RectGate> rect_gate_;

 public:
  GeoMap() = default;

  ~GeoMap() {}

  void clearAll() {
    // 2d
    polygon_.clear();
    ellipse_.clear();

    // 3d
    polyhedron_.clear();
    cylinder_.clear();
    ellipsoid_.clear();
    circle_gate_.clear();
    rect_gate_.clear();
  }

  template <class T>
  void clear(int geo_type) {
    switch (geo_type) {
      case GEO_TYPE::POLYGON:
        polygon_.clear();
        break;
      case GEO_TYPE::ELLIPSE:
        ellipse_.clear();
        break;
      case GEO_TYPE::POLYHEDRON:
        polyhedron_.clear();
        break;
      case GEO_TYPE::CYLINDER:
        cylinder_.clear();
        break;
      case GEO_TYPE::ELLIPSOID:
        ellipsoid_.clear();
        break;
      case GEO_TYPE::CIRCLEGATE:
        circle_gate_.clear();
        break;
      case GEO_TYPE::RECTGATE:
        rect_gate_.clear();
        break;
    }
  }

  // 2d
  void add(Polygon& geo_rep) { polygon_.push_back(geo_rep); }
  void add(Ellipse& geo_rep) { ellipse_.push_back(geo_rep); }

  // 3d
  void add(Polyhedron& geo_rep) { polyhedron_.push_back(geo_rep); }
  void add(Cylinder& geo_rep) { cylinder_.push_back(geo_rep); }
  void add(Ellipsoid& geo_rep) { ellipsoid_.push_back(geo_rep); }
  void add(CircleGate& geo_rep) { circle_gate_.push_back(geo_rep); }
  void add(RectGate& geo_rep) { rect_gate_.push_back(geo_rep); }

  // 2d
  void getPolygon(std::vector<Polygon>& geo_reps) { geo_reps = polygon_; }
  void getEllipse(std::vector<Ellipse>& geo_reps) { geo_reps = ellipse_; }

  // 3d
  void getPolyhedron(std::vector<Polyhedron>& geo_reps) {
    geo_reps = polyhedron_;
  }
  void getCylinder(std::vector<Cylinder>& geo_reps) { geo_reps = cylinder_; }
  void getEllipsoid(std::vector<Ellipsoid>& geo_reps) { geo_reps = ellipsoid_; }
  void getCircleGate(std::vector<CircleGate>& geo_reps) {
    geo_reps = circle_gate_;
  }
  void getRectGate(std::vector<RectGate>& geo_reps) { geo_reps = rect_gate_; }

  

  typedef shared_ptr<GeoMap> Ptr;
};

}  // namespace param_env

#endif