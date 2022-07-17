#include <mpl_planner/planner/map_planner.h>
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher prs_pub =
      nh.advertise<planning_ros_msgs::PrimitiveArray>("primitives", 1, true);
  ros::Publisher traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher refined_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "trajectory_refined", 1, true);

  std::string frame_id;
  nh.param("map/frame_id", frame_id, string("map"));


  Eigen::Matrix3Xd startState(3, N - 1), endState(3, N - 1);

  param_mpl::UniPlanner uni_mp_planner;

  
  std::unique_ptr<MPL::VoxelMapPlanner> planner_ptr;

  planner_ptr.reset(new MPL::VoxelMapPlanner(true));
  planner_ptr->setMapUtil(map_util);  // Set collision checking function
  planner_ptr->setVmax(v_max);        // Set max velocity
  planner_ptr->setAmax(a_max);        // Set max acceleration (as control input)
  planner_ptr->setYawmax(yaw_max);    // Set yaw threshold
  planner_ptr->setDt(dt);             // Set dt for each primitive
  planner_ptr->setU(U);               // Set control input
  planner_ptr->setTol(0.5);           // Tolerance for goal region
  // planner_ptr->setHeurIgnoreDynamics(true);

  // Planning thread!
  ros::Time t0 = ros::Time::now();
  bool valid = planner_ptr->plan(start, goal);

  if (!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             planner_ptr->getCloseSet().size());
  } else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             planner_ptr->getCloseSet().size());

    auto traj = planner_ptr->getTraj();
    // Publish trajectory as primitives
    planning_ros_msgs::PrimitiveArray prs_msg =
        toPrimitiveArrayROSMsg(traj.getPrimitives());
    prs_msg.header = header;
    prs_pub.publish(prs_msg);

    // Publish trajectory
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    traj_pub.publish(traj_msg);

    printf(
        "Raw traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, J(YAW): "
        "%f, total time: %f\n",
        traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
        traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());

    // Get intermediate waypoints
    auto waypoints = traj.getWaypoints();
    for (size_t i = 1; i < waypoints.size() - 1; i++)
      waypoints[i].control = Control::VEL;
    // Get time allocation
    auto dts = traj.getSegmentTimes();

    // Generate higher order polynomials
    TrajSolver3D traj_solver(Control::JRK);
    traj_solver.setWaypoints(waypoints);
    traj_solver.setDts(dts);
    traj = traj_solver.solve();

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg = toTrajectoryROSMsg(traj);
    refined_traj_msg.header = header;
    refined_traj_pub.publish(refined_traj_msg);

    printf(
        "Refined traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, "
        "J(YAW): %f, total time: %f\n",
        traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
        traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());
  }

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getCloseSet());
  // sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getValidRegion());
  ps.header = header;
  cloud_pub.publish(ps);

  // Publish location of start and goal
  sensor_msgs::PointCloud sg_cloud;
  sg_cloud.header = header;
  geometry_msgs::Point32 pt1, pt2;
  pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
  pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
  sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
  sg_pub.publish(sg_cloud);

  ros::spin();

  return 0;
}
