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
#include <vicon_env/SemanticArray.h>
#include <vicon_env/Circle.h>




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
  CIRCLE  = 0,
  ELLIPSE = 1,
  POLYGON = 2
};

std::unordered_map<int, vicon_env::Circle>  circles;
std::unordered_map<int, vicon_env::Ellipse> ellipses;
std::unordered_map<int, vicon_env::Polygon> polygons;



Eigen::Vector2i getTypeNum(std::string obs_name){

  Eigen::Vector2i type_and_num;
 
  char type = obs_name[10]; 

  if(type == 'c'){
    type_and_num(0) = SEMANTIC_TYPE::CIRCLE;
  }
  else if(type == 'e'){

    type_and_num(0) = SEMANTIC_TYPE::ELLIPSE;
  }
  else if(type == 'p'){

    type_and_num(0) = SEMANTIC_TYPE::POLYGON;

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

  std::ifstream fp(_semantic_path);
  std::string line;
  //getline(fp,line); 
  while (getline(fp,line)){ 
    std::string number;
    std::istringstream readstr(line);

    getline(readstr,number,',');
    std::cout << "number "<< number<<std::endl;
    Eigen::Vector2i type_and_num = getTypeNum(number);
    int model_id = type_and_num(1);


    switch (type_and_num(0))
    {
      case SEMANTIC_TYPE::CIRCLE:
      {
        getline(readstr,number,',');
        std::cout << "circle "<< number <<std::endl;

        vicon_env::Circle model;
        model.id = model_id;
        model.r = 0.5 * atof(number.c_str());

        std::cout << "atol(number.c_str());  " << atol(number.c_str()) << std::endl;
        std::cout << "model.r " << model.r << std::endl;


        std::pair<int, vicon_env::Circle> temp(model_id, model);
        circles.insert(temp);

        break;
      }
      case SEMANTIC_TYPE::ELLIPSE:
      {
        Eigen::Vector3d data_line;
        for(unsigned int j = 0;j <= 2;j++){
          getline(readstr,number,',');
          std::cout << "ellipse " << number <<std::endl;
          data_line(j) = atof(number.c_str());
        }

        vicon_env::Ellipse model;
        model.id = model_id;
        model.a = data_line(0);
        model.b = data_line(1);       
        model.alpha = data_line(2);
        model.r = model.b;

        std::pair<int, vicon_env::Ellipse> temp(model_id, model);
        ellipses.insert(temp);
        break;
      }
      case SEMANTIC_TYPE::POLYGON:
      {
        Eigen::Vector3d data_line;
        for(unsigned int j = 0;j <= 2;j++){
          getline(readstr,number,',');
          std::cout << "polygon " << number <<std::endl;
          data_line(j) = atof(number.c_str());
        }

        vicon_env::Polygon model;
        model.id = model_id;
        model.r = 0.5 * std::max(data_line(0), data_line(1));
        

        //get the normals
        //Eigen::Matrix2d R;
        double alpha = data_line(2) * M_PI / 180.0;
        std::cout << " alpha  " <<  alpha << std::endl;


        double a = data_line(0);
        double b = data_line(1);
        // R << std::cos(alpha), -std::sin(alpha),
        //      std::sin(alpha), std::cos(alpha);
       
        double sin_alpha = std::sin(alpha);
        double cos_alpha = std::cos(alpha);
  
        model.normals.resize(4);
        model.points.resize(4);

        model.normals[0].x = cos_alpha;
        model.normals[0].y = sin_alpha;

        model.normals[1].x = -sin_alpha;
        model.normals[1].y = cos_alpha;

        model.normals[2].x = -cos_alpha;
        model.normals[2].y = -sin_alpha;

        model.normals[3].x = sin_alpha;
        model.normals[3].y = -cos_alpha;

        model.points[0].x = a*cos_alpha;
        model.points[0].y = a*sin_alpha;

        model.points[1].x = -b*sin_alpha;
        model.points[1].y =  b*cos_alpha;

        model.points[2].x = -a*cos_alpha;
        model.points[2].y = -a*sin_alpha;

        model.points[3].x =  b*sin_alpha;
        model.points[3].y = -b*cos_alpha;

        // std::cout << "model.normals[0].x " << model.normals[0].x<<std::endl;
        // std::cout << "model.normals[0].y " << model.normals[0].y<<std::endl;
        // std::cout << "model.normals[1].x " << model.normals[1].x<<std::endl;
        // std::cout << "model.normals[1].y " << model.normals[1].y<<std::endl;
        // std::cout << "model.normals[2].x " << model.normals[2].x<<std::endl;
        // std::cout << "model.normals[2].y " << model.normals[2].y<<std::endl;
        // std::cout << "model.normals[3].x " << model.normals[3].x<<std::endl;
        // std::cout << "model.normals[3].y " << model.normals[3].y<<std::endl;


        // std::cout << "model.points[0].x " << model.points[0].x<<std::endl;
        // std::cout << "model.points[0].y " << model.points[0].y<<std::endl;
        // std::cout << "model.points[1].x " << model.points[1].x<<std::endl;
        // std::cout << "model.points[1].y " << model.points[1].y<<std::endl;
        // std::cout << "model.points[2].x " << model.points[2].x<<std::endl;
        // std::cout << "model.points[2].y " << model.points[2].y<<std::endl;
        // std::cout << "model.points[3].x " << model.points[3].x<<std::endl;
        // std::cout << "model.points[3].y " << model.points[3].y<<std::endl;


        std::pair<int, vicon_env::Polygon> temp(model_id, model);
        polygons.insert(temp);

        break;
      }
    }
  }
}


void obsCallback(const nav_msgs::Odometry &msg) {

  std::string name = msg.child_frame_id;
  Eigen::Vector2i type_and_num = getTypeNum(name);

  bool update_obs = true;

  if (_obs_num > 0 ){
    for (unsigned int i = 0; i < _obs_num; i++ ){
      if(name == obs_names.at(i)){
        update_obs = false;
        break;
      }
    }
  }


  if (update_obs){
    std::cout << "get to update obs " << name <<std::endl;
    obs_names.push_back(name);
    pcl::PointXYZ pt_obs;
    geometry_msgs::Pose pt;
    pt.orientation.w = 1.0;

    // only enable the clyinders in the forest case
    // generate polar obs
    
    double x, y, h, w;

    x = msg.pose.pose.position.y;
    y = -msg.pose.pose.position.x;
    h = 1.5;

    int heiNum = ceil(h / _resolution);
    
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    pt.position.x=  x;
    pt.position.y = y;
    pt.position.z = 0.5*h;
    // semantics_mk.pose = pt;
    // semantics_mk.scale.x = semantics_mk.scale.y = w; // less then 1
    // semantics_mk.scale.z = h;
    // //semantics_vis.markers.push_back(semantics_mk);
    // semantics_mk.id += 1;
    
    int num = type_and_num(1);

    std::cout << "type_and_num " << type_and_num <<std::endl;

    switch (type_and_num(0))
    {
      case SEMANTIC_TYPE::CIRCLE:
      {
        // this is a circle     
        auto iter = circles.find(num);
        
        vicon_env::Circle model = iter->second;

        w = model.r;

        model.pos.x = x;
        model.pos.y = y;

        std::cout << "w " << w <<std::endl;
        std::cout << "x " << x <<std::endl;
        std::cout << "y " << y <<std::endl;

        
        global_semantics_msg.circles.push_back(model);
        
        int widNum = ceil( w / _resolution) +1;

        
        for (int r = -widNum; r <= widNum; r++){
          for (int s = -widNum; s <= widNum; s++) {

            for (int t = -2.0; t < heiNum; t++) {
              pt_obs.x = x + (r + 0.5) * _resolution + 1e-2;
              pt_obs.y = y + (s + 0.5) * _resolution + 1e-2;
              pt_obs.z = (t + 0.5) * _resolution + 1e-2;

              //@yuwei: to make it as semantics
              if (  (pt_obs.x - x)*(pt_obs.x - x) + (pt_obs.y - y)*(pt_obs.y - y) > w*w/ 4.0  ){
                continue;
              }
              cloudMap.points.push_back(pt_obs);
            }
          }
        }

        std::cout << "finish" <<std::endl;

        break;
      }

      case SEMANTIC_TYPE::ELLIPSE:
      {
        
      
        break;
      }

      case SEMANTIC_TYPE::POLYGON:
      {
        auto iter = polygons.find(num);
        vicon_env::Polygon model = iter->second;


        model.pos.x = x;
        model.pos.y = y;

        w = model.r;
        std::cout << "w " << w <<std::endl;
        std::cout << "x " << x <<std::endl;
        std::cout << "y " << y <<std::endl;


        global_semantics_msg.polygons.push_back(model);

        model.points[0].x += x;
        model.points[0].y += y;

        std::cout << "model.points[0].x " << model.points[0].x<<std::endl;
        std::cout << "model.points[0].y " << model.points[0].y<<std::endl;


        model.points[1].x += x;
        model.points[1].y += y;

        model.points[2].x += x;
        model.points[2].y += y;

        model.points[3].x += x;
        model.points[3].y += y;

        
        int widNum = ceil( w / _resolution) +4;

        for (int r = -widNum; r <= widNum; r++){
          for (int s = -widNum; s <= widNum; s++) {

            pt_obs.x = x + (r + 0.5) * _resolution + 1e-2;
            pt_obs.y = y + (s + 0.5) * _resolution + 1e-2;

            Eigen::Vector2d p_, n_, pt_;

            bool valid = true;
            for (int i = 0; i < 4 ; i++){

              p_ << model.points[i].x,  model.points[i].y;
              n_ << model.normals[i].x, model.normals[i].y;
              pt_(0) = pt_obs.x;
              pt_(1) = pt_obs.y;

              std::cout << "pt_ is " << pt_ << std::endl;
              std::cout << "the checking is " <<  n_.dot(pt_ - p_) << std::endl;
              if ( n_.dot(pt_ - p_) > 1e-7){
                std::cout << "not valid "<< std::endl; 
                valid = false;
                break;
              }

            }

            if (!valid){
              continue;
            }

            for (int t = -2.0; t < heiNum; t++) {          

              pt_obs.z = (t + 0.5) * _resolution + 1e-2;

              cloudMap.points.push_back(pt_obs);
            }
          }
        }        
      
        break;
      }


    }


    _obs_num += 1;



  }
 
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = _frame_id;
  globalMap_pcd.header.stamp = ros::Time::now();
  _all_map_cloud_pub.publish(globalMap_pcd);
  _all_map_semantics_pub.publish(global_semantics_msg);


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

  _all_map_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/global_cloud", 1);

  // semantics publishers
  _all_map_semantics_pub = n.advertise<vicon_env::SemanticArray>("/global_semantics", 1);
  //_all_map_semantics_pub_vis = n.advertise<visualization_msgs::MarkerArray>("global_semantics_vis", 1);

  _all_obs_sub = n.subscribe("vicon_all_obs", 10, &obsCallback);


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
