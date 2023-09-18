#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

// for semantic map visualization
#include <visualization_msgs/MarkerArray.h>
#include <vicon_env/SemanticArray.h>


using namespace std;

vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher _all_map_cloud_pub, _all_map_semantics_pub, _all_map_semantics_pub_vis;
ros::Subscriber _all_obs_sub;

vector<double> _state;

int _obs_num = 0;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
std::string _frame_id;
bool _set_semantics = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
vicon_env::SemanticArray global_semantics_msg;

vector<string> obs_names;
std::string _semantic_path;

enum SEMANTIC_TYPE
{
  CYLINDER   = 0,
  POLYHEDRON = 1
};

bool _read_semantics = false;
std::unordered_map<int, vicon_env::Cylinder> cylinders_map;
std::unordered_map<int, vicon_env::Polyhedron> polyhedrons_map;


Eigen::Vector2i getTypeNum(std::string obs_name){
  //std::cout << "getTypeNum" << std::endl;
  Eigen::Vector2i type_and_num(-1, -1);
 
  char type = obs_name[10]; 

  if(type == 'c'){
    type_and_num(0) = SEMANTIC_TYPE::CYLINDER;
  }
  else if(type == 'p'){
    type_and_num(0) = SEMANTIC_TYPE::POLYHEDRON;
  }else{
    return type_and_num;
  }

  int obs_num = 0;
  for (int i = 11; obs_name[i] != '/' && obs_name[i] != '\0'; ++i)
  {
    obs_num *= 10;
    obs_num += obs_name[i] - '0';

  }
  type_and_num(1) = obs_num;

  return type_and_num;

}

//// the semantics are defined in the mocap frame
//// and in this code, it will convert it when sending
void ReadSemantics(){


  if (_read_semantics){
    return;
  }
  //std::cout << "ReadSemantics()" << std::endl;
  cylinders_map.clear();
  polyhedrons_map.clear();

  
  std::ifstream fp(_semantic_path);
  std::string line;
  //getline(fp,line); 
  while (getline(fp,line)){ 
    std::string number;
    std::istringstream readstr(line);

    getline(readstr,number,',');
    std::cout << "name: "<< number<<std::endl;
    Eigen::Vector2i type_and_num = getTypeNum(number);

    int model_id = type_and_num(1);

    switch (type_and_num(0))
    {
      case SEMANTIC_TYPE::CYLINDER:
      {
        Eigen::Vector2d data_line;
        for(unsigned int j = 0;j <= 1;j++){
          getline(readstr,number,',');
          data_line(j) = atof(number.c_str());
        }

        vicon_env::Cylinder model;
        model.id = model_id;
        model.r = data_line(0);
        model.h = data_line(1);

        std::cout << "model.r " << model.r << std::endl;
        std::cout << "model.h " << model.h <<std::endl;
        std::cout << "model.id " << model.id <<std::endl;

        std::pair<int, vicon_env::Cylinder> temp(model_id, model);
        cylinders_map.insert(temp);

        break;
      }
      case SEMANTIC_TYPE::POLYHEDRON:
      {
        Eigen::Vector4d data_line;
        for(unsigned int j = 0;j <= 3;j++){
          getline(readstr,number,',');
          data_line(j) = atof(number.c_str());
        }

        vicon_env::Polyhedron model;
        model.id = model_id;
        double w1 = data_line(0), w2 = data_line(1); // wide in x and y direction
        std::cout << "w1 " << w1 <<std::endl; // should be hald width 
        std::cout << "w2 " << w2 <<std::endl;
        // with a little inflation
        //get the normals
        //Eigen::Matrix2d R;
        double alpha = data_line(2) * M_PI / 180.0;
        std::cout << " alpha  " <<  alpha << std::endl;

        double h = data_line(3);
        std::cout << " height is " <<  h << std::endl;

        // R << std::cos(alpha), -std::sin(alpha),
        //      std::sin(alpha), std::cos(alpha);

        Eigen::Matrix3d rotate;
        rotate << cos(alpha), -sin(alpha), 0.0, sin(alpha), cos(alpha), 0.0, 0, 0, 1;

        Eigen::MatrixXd b_points(3, 6);
        b_points << -w1,  w1, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, -w2,  w2, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, -0.5 * h , 0.5 * h ;
        Eigen::Vector3d pos;

        model.normals.resize(6);
        model.points.resize(6);

        for (int i = 0; i < 6; i++){
          pos = rotate * b_points.col(i);
          model.points[i].x = pos(0);
          model.points[i].y = pos(1);
          model.points[i].z = 0.5 * h + pos(2);

          pos.normalize();
          model.normals[i].x = pos(0);
          model.normals[i].y = pos(1);
          model.normals[i].z = pos(2);
        }

        /***store the width in the rps***/
        model.rps.resize(1);
        model.rps[0].x = w1;
        model.rps[0].y = w2;
        model.r = alpha;
        /***store the width in the rps***/


        std::pair<int, vicon_env::Polyhedron> temp(model_id, model);
        polyhedrons_map.insert(temp);

        break;
      }
    }
  }



  _read_semantics = true;


  return;
}


void obsCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  //std::cout << "msg" << std::endl;
  //std::cout << "msg.pose.pose.position.y" << msg->pose.pose.position.y << std::endl;
  std::string name = msg->child_frame_id;
  //std::cout << "name" << name << std::endl;
  Eigen::Vector2i type_and_num = getTypeNum(name);

  if (type_and_num(0) == -1 && type_and_num(1) == -1){
    return;
  }

  bool update_obs = true;
  
  if (_obs_num > 0 ){
    for (unsigned int i = 0; i < _obs_num; i++ ){
      //std::cout << "obs_names.at(i)" << obs_names.at(i) << std::endl;
      if(name == obs_names.at(i)){
        update_obs = false;
        break;
      }
    }
  }

  if (update_obs){
    //std::cout << "get to update obs " << name <<std::endl;
    obs_names.push_back(name);
    pcl::PointXYZ pt_obs;
    //geometry_msgs::Pose pt;
    //pt.orientation.w = 1.0;
    // only enable the clyinders in the forest case
    // generate polar obs
    double x, y, w;

    x = msg->pose.pose.position.y;
    y = -msg->pose.pose.position.x;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    //pt.position.x=  x;
    //pt.position.y = y;
    //pt.position.z = 0.5*h;
    // semantics_mk.pose = pt;
    // semantics_mk.scale.x = semantics_mk.scale.y = w; // less then 1
    // semantics_mk.scale.z = h;
    // //semantics_vis.markers.push_back(semantics_mk);
    // semantics_mk.id += 1;
    
    int num = type_and_num(1);
    std::cout << "type_and_num " << type_and_num.transpose() <<std::endl;

    switch (type_and_num(0))
    {
      case SEMANTIC_TYPE::CYLINDER:
      {
        // this is a cylinder     
        auto iter = cylinders_map.find(num);
        
        if (iter == cylinders_map.end()) {
          return;
        }
        vicon_env::Cylinder model = iter->second;

        model.pos.x = x;
        model.pos.y = y;

        std::cout << "x " << x <<std::endl;
        std::cout << "y " << y <<std::endl;

        
        global_semantics_msg.cylinders.push_back(model);
        

        // publish the point cloud 
        int heiNum = ceil(model.h / _resolution);
        int widNum = ceil(model.r / _resolution) +1;
        double r_sqr = model.r * model.r;
        for (int r = -widNum; r <= widNum; r++){
          for (int s = -widNum; s <= widNum; s++){
            for (int t = -2.0; t < heiNum; t++){
              pt_obs.x = x + (r + 0.5) * _resolution + 1e-2;
              pt_obs.y = y + (s + 0.5) * _resolution + 1e-2;
              pt_obs.z =     (t + 0.5) * _resolution + 1e-2;
              //@yuwei: to make it as semantics
              if ((pt_obs.x - x)*(pt_obs.x - x) + (pt_obs.y - y)*(pt_obs.y - y) > r_sqr){
                continue;
              }
              cloudMap.points.push_back(pt_obs);
            }
          }
        }
         _obs_num += 1;
        //std::cout << "finish" <<std::endl;

        break;
      }
      case SEMANTIC_TYPE::POLYHEDRON:
      {
        auto iter2 = polyhedrons_map.find(num);

        if (iter2 == polyhedrons_map.end()) {
          return;
        }
        vicon_env::Polyhedron model = iter2->second;
        double w1 = model.rps[0].x, w2 = model.rps[0].y, alpha = model.r;
        Eigen::Matrix3d rotate;
        rotate << cos(alpha), -sin(alpha), 0.0, sin(alpha), cos(alpha), 0.0, 0, 0, 1;

        model.r = 1.1 * std::min(w1, w2);

        // std::cout << "x " << x <<std::endl;
        // std::cout << "y " << y <<std::endl;

        for (int i = 0; i < 6; i++){
          model.points[i].x += x;
          model.points[i].y += y;
        }

        int size = ceil(w1 / w2);

        // std::cout << "size is " << size << std::endl;
        // std::cout << "w1 is " << w1 << "w2 is " << w2 << std::endl;



        model.rps.resize(size);
        int idx = 0;
        for (double r = -w1 + w2; r < w1 + w2; r += 2.0 *  w2){
          model.rps[idx].x = x + cos(alpha)*r; 
          model.rps[idx].y = y + sin(alpha)*r;
          idx += 1;
        }

        global_semantics_msg.polyhedrons.push_back(model);


        // publish the point cloud 
        int heiNum = ceil(2.0 * model.points[0].z / _resolution);
        int widNum1 = ceil(w1 / _resolution) ;
        int widNum2 = ceil(w2 / _resolution) ;
        Eigen::Vector3d box_scale;

        for (int r = -widNum1; r <  widNum1; r++){
          for (int s = -widNum2; s < widNum2; s++){
            for (int t = -2.0; t < heiNum; t++){

              box_scale = rotate * Eigen::Vector3d( (r + 0.5) * _resolution, 
                                                    (s + 0.5) * _resolution,
                                                    (t + 0.5) * _resolution);
                              
              pt_obs.x = x + box_scale(0) + 1e-2;
              pt_obs.y = y + box_scale(1) + 1e-2;
              pt_obs.z =     box_scale(2) + 1e-2;

              cloudMap.points.push_back(pt_obs);
            }
          }
        } 
         _obs_num += 1;      
        std::cout << "finish" <<std::endl;
        break;
      }


    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = _frame_id;

  }
  //std::cout << "cloudMap.points.size()" << cloudMap.points.size() << std::endl;


  globalMap_pcd.header.stamp = ros::Time::now();
  _all_map_cloud_pub.publish(globalMap_pcd);
  _all_map_semantics_pub.publish(global_semantics_msg);
  //std::cout << "cloudMap.points.size()" << cloudMap.points.size() << std::endl;
  return;
}


/***
 * 
 * the mocap frame is   
 * 
 * ^ y
 * |
 * |
 * |———————— > x
 * 
 * 
 * the virtual global frame is
 * 
 *            ^ x
 *            |
 *            |
 * y <————————|
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * ***/


int main(int argc, char** argv) {

  ros::init(argc, argv, "vicon_map_node");
  ros::NodeHandle n("~");
  // point cloud publisher
  _all_map_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/global_cloud", 1);
  // semantics publisher
  _all_map_semantics_pub = n.advertise<vicon_env::SemanticArray>("/global_semantics", 1);
  //_all_map_semantics_pub_vis = n.advertise<visualization_msgs::MarkerArray>("global_semantics_vis", 1);


  _all_obs_sub = n.subscribe("vicon_all_obs", 50, &obsCallback);


  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);

  n.param("map/resolution", _resolution, 0.1);
  n.param("map/frame_id", _frame_id, string("map"));

  n.param("sensing/radius", _sensing_range, 5.0);
  n.param("sensing/rate", _sense_rate, 10.0);
  n.param("semantic_path", _semantic_path, string("case1.csv"));

  // semantics_mk.header.frame_id = _frame_id;
  // semantics_mk.header.stamp = ros::Time::now();
  // semantics_mk.type = visualization_msgs::Marker::CYLINDER;
  // semantics_mk.action  = visualization_msgs::Marker::ADD;
  // semantics_mk.id = 0;
  // semantics_mk.color.r = 0.5;
  // semantics_mk.color.g = 0.5;
  // semantics_mk.color.b = 0.5;
  // semantics_mk.color.a = 0.6;

  global_semantics_msg.mav_id = -1; // -1 for global, 0 + for the mav_id
  global_semantics_msg.header.frame_id = _frame_id;

  ReadSemantics();

  ros::Duration(0.5).sleep();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}