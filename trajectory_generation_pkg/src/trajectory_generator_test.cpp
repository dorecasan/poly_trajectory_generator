#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_trajectory_generation/segment.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <ios>
#include <iostream>
#include <string>

template <int _N = 6>
class WaypointFollower {
private:
  static constexpr int N_ = _N;

  ros::NodeHandle nh_;

  ros::Subscriber poseArraySub_;
  ros::Publisher desiredStatePub_;
  ros::Publisher markerPub_;
  ros::Publisher currentPath_;
  Eigen::Vector2d position_ = Eigen::Vector2d::Zero(); 

  ros::Timer desiredStateTimer_;
  ros::Timer desiredPathTimer_;

  ros::Time trajectoryStartTime_;
  mav_trajectory_generation::Trajectory trajectory_;
  std::vector<double> segment_times_ ;

  int D_;
  double v_max_;
  double a_max_;
  std::string save_path_;

public:
  WaypointFollower(): nh_("~") {

    nh_.param<int>("conti_derive",D_,2);
    nh_.param<double>("v_max",v_max_,20.0);
    nh_.param<double>("a_max",a_max_,20.0);
    nh_.param<std::string>("save_path",save_path_,"results.txt");

    poseArraySub_ =
        nh_.subscribe("/desired_poses", 1,
                     &WaypointFollower::generateOptimizedTrajectory, this);
    desiredStatePub_ =
        nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_trajectory", 1);
    markerPub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/desired_marker", 10);

    currentPath_ = nh_.advertise<nav_msgs::Path>("/desired_path", 1);

    desiredStateTimer_ = nh_.createTimer(
        ros::Rate(5), &WaypointFollower::publishDesiredState, this);

    desiredPathTimer_ =
        nh_.createTimer(ros::Rate(5), &WaypointFollower::publishPath, this);

    desiredPathTimer_.start();
    desiredStateTimer_.start();
  }

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const &poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory_.clear();
      return;
    }
    if (!trajectory_.empty())
      return;


    mav_trajectory_generation::Vertex start_position(D_), end_position(D_);
    mav_trajectory_generation::Vertex::Vector vertices;
    // const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    const int derivative_to_optimize = 2;


    position_ << poseArray.poses[0].position.x , poseArray.poses[0].position.y;
    
    using namespace mav_trajectory_generation::derivative_order;
    start_position.makeStartOrEnd(position_, derivative_to_optimize);
    vertices.push_back(start_position);
    int n = poseArray.poses.size();
    Eigen::Vector2d last_vertex;
    for (auto i = 1; i < n-1; ++i) {
 
      Eigen::Vector2d current_vertex;
      current_vertex << poseArray.poses[i].position.x, poseArray.poses[i].position.y;
      mav_trajectory_generation::Vertex middle_vertex(D_);
      middle_vertex.addConstraint(POSITION, current_vertex);
      vertices.push_back(middle_vertex);
      
    }
    
    last_vertex << poseArray.poses[n-1].position.x, poseArray.poses[n-1].position.y;
    end_position.makeStartOrEnd(last_vertex, derivative_to_optimize);
    vertices.push_back(end_position);
    
    segment_times_ = estimateSegmentTimes(vertices, v_max_, a_max_);
  

    mav_trajectory_generation::PolynomialOptimization<N_> opt(D_);
    opt.setupFromVertices(vertices, segment_times_, derivative_to_optimize);
    opt.solveLinear();

    mav_trajectory_generation::Segment::Vector segments;
    //        opt.getSegments(&segments); // Unnecessary?
    opt.getTrajectory(&trajectory_);
    trajectoryStartTime_ = ros::Time::now();

    writeSegmentParamsToFile(save_path_);

    ROS_INFO("Generated optimizes trajectory from %d waypoints",(int)vertices.size());
  }

  void writeSegmentParamsToFile(std::string file_name){
    
    std::ofstream out(file_name);
    
    mav_trajectory_generation::Segment::Vector* segments = new mav_trajectory_generation::Segment::Vector;
    
    trajectory_.getSegments(segments);
    segment_times_  = trajectory_.getSegmentTimes();
    
    out << "Total: " <<segments->size() <<" segments"<< std::endl;
    out << "Segment times: " << "["; 
    for (auto t: segment_times_){
      out << t <<" ";
    }
    out <<"]"<< std::endl;
    out << *segments;

  }
  

  void publishDesiredState(ros::TimerEvent const &ev) {
    if (trajectory_.empty())
      return;
    
    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;

    ros::Duration t = ros::Time::now() - trajectoryStartTime_;
    next_point.time_from_start = t;         
                                            
    double sampling_time = t.toSec() + 0.5; 
    ROS_INFO("\nSampling time %f\n", sampling_time);
    if (sampling_time > trajectory_.getMaxTime())
      sampling_time = trajectory_.getMaxTime();


    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector2d des_position = trajectory_.evaluate(sampling_time, POSITION);
    Eigen::Vector2d des_velocity = trajectory_.evaluate(sampling_time, VELOCITY);
    Eigen::Vector2d des_accel = trajectory_.evaluate(sampling_time, ACCELERATION);

    ROS_INFO("Traversed %f percent of the trajectory.",sampling_time / trajectory_.getMaxTime() * 100);


    next_point.transforms.resize(1);
    next_point.transforms[0].translation.x = des_position.x();
    next_point.transforms[0].translation.y = des_position.y();
    next_point.transforms[0].translation.z = 0;
    next_point.velocities.resize(1);
    next_point.velocities[0].linear.x = des_velocity.x();
    next_point.velocities[0].linear.y = des_velocity.y();
    next_point.velocities[0].linear.z = 0;
    next_point.accelerations.resize(1);
    next_point.accelerations[0].linear.x = des_accel.x();
    next_point.accelerations[0].linear.y = des_accel.y();
    next_point.accelerations[0].linear.z = 0;
    next_point.velocities[0].angular.x = 0;
    next_point.velocities[0].angular.y = 0;
    next_point.velocities[0].angular.z = 0;
    next_point.accelerations[0].angular.x = 0;
    next_point.accelerations[0].angular.y = 0;
    next_point.accelerations[0].angular.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    next_point.transforms[0].rotation.x = quat.x();
    next_point.transforms[0].rotation.y = quat.y();
    next_point.transforms[0].rotation.z = quat.z();
    next_point.transforms[0].rotation.w = quat.w();

    desiredStatePub_.publish(next_point);

    visualization_msgs::MarkerArray markers;
    double distance = 1.0; 
                          
    std::string frame_id = "map";

 
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance, frame_id,
                                                 &markers);
  
    markerPub_.publish(markers);

  }

  void publishPath(ros::TimerEvent const &) {
    if (trajectory_.empty())
      return;
    nav_msgs::Path toanDesiredPath;
    toanDesiredPath.header.frame_id = "map";
    double t_start = 0.0;
    double t_end = std::accumulate(segment_times_.begin(), segment_times_.end(), 0.0);;
    double dt = 0.2;
    std::vector<Eigen::VectorXd> result;
    std::vector<double> sampling_times; 
    trajectory_.evaluateRange(
        t_start, t_end, dt,
        mav_trajectory_generation::derivative_order::POSITION, &result,
        &sampling_times);

    geometry_msgs::PoseStamped toan;
    toan.header.frame_id = "map";
    for (const auto &pos : result) {
      geometry_msgs::PoseStamped toanPose;
      toanPose = toan;
      toanPose.pose.position.x = pos[0];
      toanPose.pose.position.y = pos[1];
      toanPose.pose.position.z = 0;
      toanDesiredPath.poses.push_back(toanPose);
    }
    currentPath_.publish(toanDesiredPath);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_generation_node");

  WaypointFollower<6> waypointFollower;

  ros::spin();
  return 0;
}
