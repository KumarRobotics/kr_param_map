#ifndef _CLOSED_SHAPES_HPP
#define _CLOSED_SHAPES_HPP

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

  // help functions
  Eigen::Matrix3d eulerToRot(Eigen::Vector3d &odom_euler)
  {

    Eigen::Vector3d rpy(odom_euler(0), odom_euler(1), odom_euler(2));
    Eigen::Quaternion<double> qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
    Eigen::Quaternion<double> qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
    Eigen::Quaternion<double> qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
    Eigen::Matrix3d R = Eigen::Matrix3d(qz * qy * qx);

    return R;
  }



  /* ---------------- 2D shapes ---------------- */


  class Polygon
  {
  private:

    // h-representation = [pt^T, outter_normal^T]^T
    Eigen::MatrixX4d hPoly_;

  public:

    Polygon() = default;
    
    Polygon(const Eigen::MatrixX4d &hPoly) : hPoly_(hPoly) {}

    ~Polygon() {}

    // check if the point is inside the Polygon
    bool is_inside(const Eigen::Vector2d &pt, 
                   const double epsilon = 1.0e-6)
    {
      Eigen::Vector2d p_, n_;
      for (int i = 0; i < hPoly_.cols(); i++)
      {
        p_ = hPoly_.col(i).head<2>();
        n_ = hPoly_.col(i).tail<2>();
        if ( n_.dot(pt- p_) > epsilon)
        {
          return false;
        }
      }
      return true;
    }

  };

  //--------------- Non-polygon
  class Ellipse
  {
  private:
  
    Eigen::Matrix2d E_; // 2*2 matrix
    Eigen::Vector2d d_;

  public:

    Ellipse() = default;
    
    Ellipse(const Eigen::Matrix2d &E, Eigen::Vector2d d) : E_(E), d_(d) {}

    ~Ellipse() {}

    // Check if the point is inside
    bool is_inside(const Eigen::Vector2d &pt)
    {
      return (E_.inverse() * (pt - d_)).norm() <= 1.0;
    }

  };


  /* ---------------- 3D shapes ---------------- */

  // The h-representation of polyhedron (Hyperplane)
  class Polyhedron
  {
  private:

    // h-representation = [outter_normal^T, pt^T]^T
    Eigen::MatrixXd hPoly_;

  public:

    Polyhedron() {}

    Polyhedron(const Eigen::MatrixXd &hPoly) : hPoly_(hPoly) {}

    ~Polyhedron() {}

    // check if the point is inside the Polyhedron
    bool is_inside(Eigen::Vector3d &pt,
                   const double epsilon = 1.0e-6)
    {
      Eigen::Vector3d p_, n_;
      for (int i = 0; i < hPoly_.cols(); i++)
      {
        p_ = hPoly_.col(i).head<3>();
        n_ = hPoly_.col(i).tail<3>();
        if ( n_.dot(pt- p_) > epsilon)
        {
          return false;
        }
      }
      return true;
    }

    // for point cloud visualization
    void get_vis_pts(pcl::PointCloud<pcl::PointXYZ> &clouds, 
                     const double res = 0.1)
    {

    }



    //randomly generate a convex polytope
    //given the center point and the boundary
    void random_init(Eigen::Vector3d &cpt,
                     Eigen::Vector3d &bound)
    {

      Eigen::Matrix3Xd mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vertices;

      // Randomly generate a set of points
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> x_dis(-bound(0), bound(0));
      std::uniform_real_distribution<> y_dis(-bound(1), bound(1));
      std::uniform_real_distribution<> z_dis(-bound(2), bound(2));
      std::uniform_real_distribution<> vs_num(4, 35);

      int rand_num = ceil(vs_num(gen));
      vertices.resize(3, rand_num);
      for (int i = 0; i < rand_num; i++)
      {
        vertices.col(i) << cpt(0) + x_dis(gen), 
                           cpt(1) + y_dis(gen),
                           cpt(2) + z_dis(gen);
      }

      // Get the convex hull in mesh
      quickhull::QuickHull<double> qh;
      const auto cvxHull = qh.getConvexHull(vertices.data(),
                                            vertices.cols(),
                                            false, false);
      const auto &idBuffer = cvxHull.getIndexBuffer();
      const auto &vtBuffer = cvxHull.getVertexBuffer();
      int ids = idBuffer.size();
      mesh.resize(3, ids);
      quickhull::Vector3<double> v;
      for (int i = 0; i < ids; i++)
      {
        v = vtBuffer[idBuffer[i]];
        mesh(0, i) = v.x;
        mesh(1, i) = v.y;
        mesh(2, i) = v.z;
      }

      Eigen::MatrixXd hPoly(6, ids / 3);
      Eigen::Vector3d normal, point, edge0, edge1;
      for (int i = 0; i < ids / 3; i++)
      {
          point = mesh.col(3 * i + 1);
          edge0 = point - mesh.col(3 * i);
          edge1 = mesh.col(3 * i + 2) - point;
          normal = edge0.cross(edge1).normalized();
          hPoly.col(i).head<3>() = point;
          hPoly.col(i).tail<3>() = normal;
      }

      hPoly_ = hPoly;
      //std::cout << "hPoly_ is  " << hPoly_ << std::endl;

    }

              
  };


  //--------------- Non-polyhedron
  class Cylinder // nomral cylinder
  {
  private:
  
    Eigen::Vector3d cpt_; // 3*3 matrix
    double r_;

  public:

    Cylinder() = default;
    
    Cylinder(const Eigen::Vector3d &cpt, double &r) : cpt_(cpt), r_(r) {}

    ~Cylinder() {}

    // Check if the point is inside
    bool is_inside(const Eigen::Vector3d &pt)
    {
      if (pt(2) > cpt_(2) * 2)
      {
        return false;
      }else if ( ((pt - cpt_).head(2)).norm() > r_)
      {
        return false;
      }
      return true;
    }

  };

  class Ellipsoid
  {
  private:
  
    Eigen::Matrix3d E_; // 3*3 matrix
    Eigen::Vector3d d_;

  public:

    Ellipsoid() = default;
    
    Ellipsoid(const Eigen::Matrix3d &E, Eigen::Vector3d d) : E_(E), d_(d) {}

    ~Ellipsoid() {}

    // Check if the point is inside
    bool is_inside(const Eigen::Vector3d &pt)
    {
      return (E_.inverse() * (pt - d_)).norm() <= 1.0;
    }

  };


}

#endif