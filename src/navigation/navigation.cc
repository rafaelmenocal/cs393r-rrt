//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include "navigation.h"

#include <cstdlib>
#include <cmath>
#include <chrono>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/random.h"
#include "visualization/visualization.h"
#include "node.h"

// #include "global_planner.h"
// #include "graph.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2d;
using geometry::MinDistanceLineArc;
using namespace std;

using namespace math_util;
using namespace ros_helpers;

namespace {
  ros::Publisher drive_pub_; // publish to /ackermann_curvature_drive
  ros::Publisher viz_pub_;  // publish to /visualization
  VisualizationMsg local_viz_msg_; // points, lines, arcs, etc.
  VisualizationMsg global_viz_msg_; // points, lines, arcs, etc.
  AckermannCurvatureDriveMsg drive_msg_; // velocity, curvature
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-5;
  float critical_time = 0.0;
  float critical_dist = 0.0;
  float speed = 0.0;
  float accel = 0.0;
  bool collision = false;
  float del_angle_ = 0.0;
  std::vector<Vector2f> proj_point_cloud_;
  std::vector<Vector2f> drawn_point_cloud_;
  Eigen::Vector2f local_target = Vector2f(0.0,0.0);
  Eigen::Vector2f global_target;
  int odd_num_paths = 15; // make sure this is odd
  double previous_time;
  double current_time;
  double del_time;
  bool obstacle_debug = false;
  bool odometry_debug = false;

  float map_x_min = -50.0;
  float map_x_max = 50.0; 
  float map_y_min = -50.0;  
  float map_y_max = 50.0;  

  // std::map<std::string, Node*> graph;
  bool sampled = false;
  // bool start_intialized = false;
  Eigen::Vector2f sampled_point(0.0, 0.0);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  util_random::Random rng_(seed);
} //namespace

namespace navigation {


// -------- BEGIN CODE FOR OBSTACLE AVOIDANCE -----------------
float VelocityToSpeed(const Vector2f& velocity) {
  return sqrt(pow(velocity.x(), 2) + pow(velocity.y(), 2));
}

float VectorAccelToAccel(const Vector2f& accel) {
  return sqrt(pow(accel.x(), 2) + pow(accel.y(), 2));
}

std::vector<Vector2f> ProjectPointCloud1D(const std::vector<Vector2f>& point_cloud_,
                                           const Vector2f& velocity, float speed,
                                           float critical_time, float latency){
  vector<Vector2f> proj_point_cloud_;
  for (const auto& point : point_cloud_){
    Vector2f proj_point = point - (critical_time + latency) * velocity;
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

std::vector<Vector2f> ProjectPointCloud2D(const std::vector<Vector2f>& point_cloud_,
                                          const Vector2f& velocity, float speed, float critical_time,
                                          float latency, float angle, float angular_vel_, float curvature){
  vector<Vector2f> proj_point_cloud_;
  Eigen::Rotation2Df rot(-angle);
  for (const auto& point : point_cloud_){
    Vector2f proj_point = (rot * (point - (critical_time + latency) * Vector2f(speed, 0.0)));
    proj_point_cloud_.push_back(proj_point);
  }
  return proj_point_cloud_;
}

bool PointWithinSafetyMargin(const Vector2f& proj_point,
                             float width, float length,
                             float axle_offset, float safety_margin_front, float safety_margin_side) {
  bool within_length = (proj_point.x() < (axle_offset + length + safety_margin_front)) && (proj_point.x() > (axle_offset - safety_margin_front));
  bool within_width = (proj_point.y() < (safety_margin_side + width/2.0)) && (proj_point.y() > (-safety_margin_side - width/2.0));
  return within_length && within_width;
}

bool ProjectedPointCloudCollision(const std::vector<Vector2f>& proj_point_cloud_,
                                  float width, float length, float axle_offset,
                                  float safety_margin_front, float safety_margin_side) {
  for (const auto& projected_point: proj_point_cloud_){
    if (PointWithinSafetyMargin(projected_point, width, length, axle_offset, safety_margin_front, safety_margin_side)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(projected_point, 0xeb3434, local_viz_msg_);
      if (obstacle_debug) {ROS_INFO("Collision Alert: Collision Detected!");}
      return true;
    }
  }
  if (obstacle_debug) {ROS_INFO("Collision Alert: None");}
  return false;
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomVelocity(const Vector2f& last_loc, const Vector2f& current_loc, float update_freq) {
  return update_freq * Vector2f(current_loc.x() - last_loc.x(), current_loc.y() - last_loc.y());
}

// Given the previous and current odometry readings
// return the instantaneous velocity
Vector2f GetOdomAcceleration(const Vector2f& last_vel, const Vector2f& current_vel, float update_freq) {
  return (last_vel - current_vel) / update_freq;
}

void PrintPaths(object_avoidance::paths_ptr paths){
  ROS_INFO("----------------------");
  for (const auto& path : *paths){
    ROS_INFO("c= %f, fpl= %f, cl= %f s= %f", path.curvature,  path.free_path_lengthv2, path.clearance, path.score);
  }
}

void DrawPaths(object_avoidance::paths_ptr paths){
  for (auto& path : *paths){
    visualization::DrawPathOption(path.curvature, path.free_path_lengthv2, 0.0, local_viz_msg_);
  }
}

float_t CalculateVelocityMsg(const std::vector<Vector2f>& point_cloud_, object_avoidance::CarSpecs car_specs_, float_t free_path_length, float_t critical_dist, float_t max_vel) {
  for (const auto& point: point_cloud_){
    if (PointWithinSafetyMargin(point, car_specs_.car_width, car_specs_.car_length, car_specs_.rear_axle_offset, car_specs_.car_safety_margin_front, car_specs_.car_safety_margin_side)){
      //draw a red point where identified point is within safety margin
      visualization::DrawPoint(point, 0xeb3434, local_viz_msg_);
      if (obstacle_debug) {ROS_INFO("Collision Alert: Collision Detected!");}
      collision = true;
      return 0.0;
    }
  }

  collision = false;
  if (free_path_length <= critical_dist){
    if (obstacle_debug) {ROS_INFO("Collision Alert: Predicting Collision");}
    return 0.0;
  } else {
    if (obstacle_debug) {ROS_INFO("Collision Alert: None");}
    return max_vel;
  }
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {

  if (odom_initialized_) {
    last_odom_loc_ = odom_loc_;
    last_odom_angle_ = odom_angle_;
  }

  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  odom_loc_ = loc;
  odom_angle_ = angle;
  current_time = ros::Time::now().toSec();

  if (odom_initialized_) {
    del_time = current_time - previous_time;
    last_odom_vel_ = odom_vel_;
    odom_vel_ = GetOdomVelocity(last_odom_loc_, odom_loc_, 1.0 / del_time); //update_frequency_);
    odom_accel_ = GetOdomAcceleration(last_odom_vel_, odom_vel_, 1.0 / del_time); //update_frequency_);
    speed = VelocityToSpeed(odom_vel_);
    accel = VectorAccelToAccel(odom_accel_);
    del_angle_ = odom_angle_ - last_odom_angle_;
    odom_omega_ = del_angle_ / del_time;
    critical_time =  speed / max_accel_;
    critical_dist = 0.5 * (critical_time + latency) * max_vel_;

    if (odometry_debug){
      ROS_INFO("================START CONTROL================="); 
      ROS_INFO("current_time = %f", current_time);   
      ROS_INFO("previous_time = %f", previous_time);  
      ROS_INFO("del_time = %f", del_time); 
      ROS_INFO("odom_loc_ = (%f, %f)", odom_loc_.x(), odom_loc_.y());
      ROS_INFO("last_odom_loc_ = (%f, %f)", last_odom_loc_.x(), last_odom_loc_.y());
      ROS_INFO("odom_vel_ = (%f, %f)", odom_vel_.x(),odom_vel_.y());
      ROS_INFO("last_odom_vel = (%f, %f)",last_odom_vel_.x(), last_odom_vel_.y());
      ROS_INFO("odom_accel_ = (%f, %f)", odom_accel_.x(), odom_accel_.y());    
      ROS_INFO("speed = %f", speed);
      ROS_INFO("accel = %f", accel);
      ROS_INFO("latency = %f", latency);
      ROS_INFO("critical_time = %f", critical_time);
      ROS_INFO("critical_dist = %f", critical_dist);
      ROS_INFO("----------------------");
      ROS_INFO("odom_angle_ = %f", odom_angle_);
      ROS_INFO("last_odom_angle_ = %f", last_odom_angle_);
      ROS_INFO("del_angle_ = %f", del_angle_);
      ROS_INFO("odom_omega_ = %f", odom_omega_);
      ROS_INFO("----------------------");
    }
  }

  previous_time = current_time;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::DrawCar(){
    visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_, car_safety_margin_front_, car_safety_margin_side_, drive_msg_, local_viz_msg_, collision);
}

void Navigation::ObstacleAvoid(){
  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) {
    return;
  }

  // -------START CONTROL---------------------------------------
  path_planner_->UpdatePaths(point_cloud_, local_target);
  auto best_path = path_planner_->GetHighestScorePath(); //GetPlannedCurvature();
  drive_msg_.curvature = best_path.curvature;
  drive_msg_.velocity = CalculateVelocityMsg(point_cloud_, car_specs_, best_path.free_path_lengthv2, critical_dist, max_vel_);
  if (local_target == Eigen::Vector2f(0.0,0.0)){
      drive_msg_.curvature = 0.0;
      drive_msg_.velocity = 0.0;
  }
  // ---------- Visualizations & Terminal Outputs -----------------
  
  DrawPaths(path_planner_->GetPaths());
  DrawCar();
  // visualization::DrawLocalTarget(local_target, local_viz_msg_);
  // visualization::DrawPathOption(drive_msg_.curvature, local_target_radius, 0, local_viz_msg_);
  
  if (obstacle_debug) {ROS_INFO("drive_msg_.velocity = %f", drive_msg_.velocity);}
  if (obstacle_debug) {ROS_INFO("drive_msg_.curvature = %f", drive_msg_.curvature);}

  // Display Driving Status
  if ((drive_msg_.velocity == 0) && (speed == 0.0)){
    if (obstacle_debug) {ROS_INFO("Status: Stopped");}
  }
  else if (drive_msg_.velocity == 0) {
    if (obstacle_debug) {ROS_INFO("Status: Stopping");}
  }
  else { // driving
    if (drive_msg_.curvature == 0) { 
      if (obstacle_debug) {ROS_INFO("Status: Driving Straight");}
    } else if (drive_msg_.curvature > 0){
      if (obstacle_debug) {ROS_INFO("Status: Turning Left");}
    }
      else if (drive_msg_.curvature < 0){
      if (obstacle_debug) {ROS_INFO("Status: Turning Right");}
    }
  }

  if (obstacle_debug) {PrintPaths(path_planner_->GetPaths());}
  if (obstacle_debug) {ROS_INFO("=================END CONTROL==================");}  
}
// -------- END CODE FOR OBSTACLE AVOIDANCE -----------------

// -------- BEGIN CODE FOR RRT NAVIGATION -----------------
void InitializeGraph(){
  // add start and end point to graph
  // if (!start_initialized){
  //   std::string id = to_string(robot_loc_.x()) + "," + to_string(robot_loc_.x()) + "," to_string(robot_angle_);
  //   Node* start_node = new Node();
  //   start_node->x = robot_loc_.x();
  //   start_node->y = robot_loc_.y();
  //   start_node->theta = robot_angle_;
  //   start_node->id = id;
  //   graph[id] = start_node;
  //   start_initialized = true;
  // }
}

// Navigation Constructor called when Navigation instantiated in navigation_main.cc
Navigation::Navigation(const string& map_file, const double& latency, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    odom_omega_(0),
    latency(latency) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  ros::Time::init();
  map_.Load(map_file);
  InitializeGraph();

  // Create ObjectAvoidance object that will manage path calculations
  path_planner_.reset(
    new object_avoidance::ObjectAvoidance(car_specs_, odd_num_paths, min_turn_radius_));
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  ROS_INFO("Goal Target = (%f, %f, %f)", loc.x(), loc.y(), angle);
  // for debugging to select a new sampled_point
  sampled = false;
  // add the nav_goal_loc_ as the goal_node to the graph
  // update its path using nearest nodes, if possible
}

Eigen::Vector2f SamplePointFromMap() {
  float x_val = rng_.UniformRandom(map_x_min, map_x_max);
  float y_val = rng_.UniformRandom(map_y_min, map_y_max);
  ROS_INFO("Sampled Point = (%f, %f)", x_val, y_val);
  return Eigen::Vector2f(x_val, y_val);
}


void Navigation::DrawSample() {
  visualization::DrawCross(sampled_point, 0.15, 0x0045cf, global_viz_msg_);
}

void Navigation::DrawTarget() {
  visualization::DrawCross(nav_goal_loc_, 0.15, 0x68ad7b, global_viz_msg_);
  visualization::DrawParticle(nav_goal_loc_, nav_goal_angle_, global_viz_msg_);
  // visualization::DrawArrow(nav_goal_loc_, nav_goal_angle_, 0x68ad7b, global_viz_msg);
}

void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // debuging code
  if (!sampled){
    sampled_point = SamplePointFromMap();
    sampled = true;
  } else{
      DrawSample();
  }

  drive_msg_.velocity = 0.0;
  DrawCar();
  DrawTarget();

  // // is it kinematically feasible to get to goal in graph
  // while (!graph.PathToGoalFound) {
  //   sampled_point = SamplePointFromMap();
  //   // Find nearest node that is kinodynmically feasible
  //   // if there isn't one, sample again
  //   // return only kinematically feasible parent
  //   parent = graph.getClosestNode(sampled_point.x(), sampled_point.y());
    
  //   // calculate the curvature from a given parent node to 
  //   // the sampled point
  //   // TODO(alex)
  //   curvature = CalculateCurvatureToPoint(sampled_point, parent);
    
  //   // calculate the point along the arc to the sampled_point
  //   // up to a max distance away
  //   // TODO(alex)
  //   truncated_point = TruncatePointAlongCurvature(sample_point, curvature);
    
  //   // Detect collisoin between parent node and truncated_point
  //   // if there is a detected collision, sample again or pick 
  //   // a new parent? (this can return (0.0,0.0) if a collision
  //   // is detected)
  //   // TODO(alex)
  //   truncated_point = DetectCollision(parent, map_);
    
  //   // calculate the heading of the robot if moving from
  //   // parent node to truncated_point along curavture
  //   theta = CalculateThetaAtPoint(truncated_point, parent, curvature);

  //   // create new node with calculated parameters
  //   Node node(id, parent, curvature, theta, etc.);

  //   // add node to graph
  //   graph.add(node);
  // }

  // ROS_INFO("Found Path to Goal");

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
