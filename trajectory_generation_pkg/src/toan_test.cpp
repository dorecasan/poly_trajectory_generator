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

#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WaypointFollower {
  [[maybe_unused]] ros::Subscriber currentStateSub;
  [[maybe_unused]] ros::Subscriber poseArraySub;
  ros::Publisher desiredStatePub;
  ros::Publisher markerPub;
  ros::Publisher currentDes;

  // Current state
  Eigen::Vector3d x = Eigen::Vector3d::Zero(); // current position of the UAV's c.o.m. in the world frame

  ros::Timer desiredStateTimer;
  ros::Timer toanTimer;

  ros::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const &poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty())
      return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2021  - Lab 4 coding assignment (35 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  We are using the mav_trajectory_generation library
    //  (https://github.com/ethz-asl/mav_trajectory_generation) to perform
    //  trajectory optimization given the waypoints (based on the position and
    //  orientation of the gates on the race course).
    //  We will be finding the trajectory for the position and the trajectory
    //  for the yaw in a decoupled manner.
    //  In this section:
    //  1. Fill in the correct number for D, the dimension we should apply to
    //  the solver to find the positional trajectory
    //  2. Correctly populate the Vertex::Vector structure below (vertices,
    //  yaw_vertices) using the position of the waypoints and the yaw of the
    //  waypoints respectively
    //
    //  Hints:
    //  1. Use vertex.addConstraint(POSITION, position) where position is of
    //  type Eigen::Vector3d to enforce a waypoint position.
    //  2. Use vertex.addConstraint(ORIENTATION, yaw) where yaw is a double
    //  to enforce a waypoint yaw.
    //  3. Remember angle wraps around 2 pi. Be careful!
    //  4. For the ending waypoint for position use .makeStartOrEnd as seen with
    //  the starting waypoint instead of .addConstraint as you would do for the
    //  other waypoints.
    //
    // ~~~~ begin solution

    const int D = 3; // dimension of each vertex in the trajectory
    mav_trajectory_generation::Vertex start_position(D), end_position(D);
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start_yaw(1), end_yaw(1);
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    // ============================================
    // Convert the pose array to a list of vertices
    // ============================================

    // Start from the current position and zero orientation
    using namespace mav_trajectory_generation::derivative_order;
    start_position.makeStartOrEnd(x, SNAP);
    vertices.push_back(start_position);
    start_yaw.addConstraint(ORIENTATION, 0);
    yaw_vertices.push_back(start_yaw);

    double last_yaw = 0;
    Eigen::Vector3d last_vertex;
    for (auto i = 0; i < poseArray.poses.size(); ++i) {
      // Populate vertices (for the waypoint positions)
      //
      //
      //     **** FILL IN HERE ***
      Eigen::Vector3d current_vertex;
      current_vertex << poseArray.poses[i].position.x,
          poseArray.poses[i].position.y, poseArray.poses[i].position.z;
      mav_trajectory_generation::Vertex middle_vertex(D);
      middle_vertex.addConstraint(POSITION, current_vertex);
      vertices.push_back(middle_vertex);
      last_vertex = current_vertex;
      //
      //
      // Populate yaw_vertices (for the waypoint yaw angles)
      //
      //
      //     **** FILL IN HERE ***
      double yaw = tf::getYaw(poseArray.poses[i].orientation);
      // double yaw_diff = yaw - last_yaw;
      // yaw = last_yaw + std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));

      if (i > 0) {
        while (yaw < last_yaw - M_PI) {
          yaw += 2 * M_PI;
        }
        while (yaw > last_yaw + M_PI) {
          yaw -= 2 * M_PI;
        }
      }

      mav_trajectory_generation::Vertex middle_yaw_vertex(1);
      middle_yaw_vertex.addConstraint(ORIENTATION, yaw);
      yaw_vertices.push_back(middle_yaw_vertex);

      last_yaw = yaw;
      //
      //
    }

    // end_position.makeStartOrEnd(last_vertex, SNAP);
    // vertices.push_back(end_position);
    // end_yaw.addConstraint(ORIENTATION, 0);
    // yaw_vertices.push_back(end_yaw);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    // HINT: play with these segment times and see if you can finish
    // the race course faster!
    std::vector<double> segment_times;
    const double v_max = 20.0;
    const double a_max = 20.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    std::cout << "\nSegment_times: ====>>>>>>\n";
    for (const auto &i : segment_times) {
      std::cout << "  " << i;
    }
    std::cout << "   \n";

    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, SNAP);
    opt.solveLinear();

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    yaw_opt.solveLinear();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    //        opt.getSegments(&segments); // Unnecessary?
    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = ros::Time::now();

    ROS_INFO("Generated optimizes trajectory from %d waypoints",
             vertices.size());
  }

  void publishPath(ros::TimerEvent const &) {
    if (trajectory.empty())
      return;
    nav_msgs::Path toanDesiredPath;
    toanDesiredPath.header.frame_id = "world";
    double t_start = 0.2;
    double t_end = 50.0;
    double dt = 0.2;
    std::vector<Eigen::VectorXd> result;
    std::vector<double> sampling_times; // Optional.
    trajectory.evaluateRange(
        t_start, t_end, dt,
        mav_trajectory_generation::derivative_order::POSITION, &result,
        &sampling_times);

    geometry_msgs::PoseStamped toan;
    toan.header.frame_id = "world";
    for (const auto &pos : result) {
      geometry_msgs::PoseStamped toanPose;
      toanPose = toan;
      toanPose.pose.position.x = pos[0];
      toanPose.pose.position.y = pos[1];
      toanPose.pose.position.z = pos[2];
      toanDesiredPath.poses.push_back(toanPose);
    }
    currentDes.publish(toanDesiredPath);
  }
  void publishDesiredState(ros::TimerEvent const &ev) {
    if (trajectory.empty())
      return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2021  - Lab 4 coding assignment (15 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Finally we get to send commands to our controller! First fill in
    //  properly the value for 'nex_point.time_from_start' and 'sampling_time'
    //  (hint: not 0) and after extracting the state information from our
    //  optimized trajectory, finish populating next_point.
    //
    // ~~~~ begin solution
    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;
    ros::Duration t = ros::Time::now() - trajectoryStartTime;
    next_point.time_from_start = t;         // <--- Correct
    double sampling_time = t.toSec() + 0.5; // <--- Correct this
    ROS_INFO("\nSampling time toan dai ca: current: %f, future: %f\n",
             t.toSec(), sampling_time);
    if (sampling_time > trajectory.getMaxTime())
      sampling_time = trajectory.getMaxTime();

    // Getting the desired state based on the optimized trajectory we found.
    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector3d des_position = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d des_velocity = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d des_accel =
        trajectory.evaluate(sampling_time, ACCELERATION);
    Eigen::VectorXd des_orientation =
        yaw_trajectory.evaluate(sampling_time, ORIENTATION);
    ROS_INFO("Traversed %f percent of the trajectory.",
             sampling_time / trajectory.getMaxTime() * 100);

    // Populate next_point
    //
    //
    next_point.transforms.resize(1);
    next_point.transforms[0].translation.x = des_position.x();
    next_point.transforms[0].translation.y = des_position.y();
    next_point.transforms[0].translation.z = des_position.z();
    next_point.velocities.resize(1);
    next_point.velocities[0].linear.x = des_velocity.x();
    next_point.velocities[0].linear.y = des_velocity.y();
    next_point.velocities[0].linear.z = des_velocity.z();
    next_point.accelerations.resize(1);
    next_point.accelerations[0].linear.x = des_accel.x();
    next_point.accelerations[0].linear.y = des_accel.y();
    next_point.accelerations[0].linear.z = des_accel.z();
    next_point.velocities[0].angular.x = 0;
    next_point.velocities[0].angular.y = 0;
    next_point.velocities[0].angular.z = 0;
    next_point.accelerations[0].angular.x = 0;
    next_point.accelerations[0].angular.y = 0;
    next_point.accelerations[0].angular.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, des_orientation[0]);
    next_point.transforms[0].rotation.x = quat.x();
    next_point.transforms[0].rotation.y = quat.y();
    next_point.transforms[0].rotation.z = quat.z();
    next_point.transforms[0].rotation.w = quat.w();
    //
    //
    desiredStatePub.publish(next_point);

    visualization_msgs::MarkerArray markers;
    double distance = 1.0; // Distance by which to seperate additional markers.
                           // Set 0.0 to disable.
    std::string frame_id = "world";

    // From Trajectory class:
    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id,
                                                 &markers);

    markerPub.publish(markers);
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

public:
  visualization_msgs::MarkerArray markers_toan;
  explicit WaypointFollower(ros::NodeHandle &nh) {
    poseArraySub =
        nh.subscribe("/desired_traj_vertices", 1,
                     &WaypointFollower::generateOptimizedTrajectory, this);
    desiredStatePub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    markerPub =
        nh.advertise<visualization_msgs::MarkerArray>("/toan_marker", 10);
    currentDes = nh.advertise<nav_msgs::Path>("/toan_current_des", 1);
    desiredStateTimer = nh.createTimer(
        ros::Rate(5), &WaypointFollower::publishDesiredState, this);
    toanTimer =
        nh.createTimer(ros::Rate(5), &WaypointFollower::publishPath, this);
    desiredStateTimer.start();
    toanTimer.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}
