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
  bool process_sample_debug = false;

  float map_x_min = -44.0;
  float map_x_max = 44.0; 
  float map_y_min = -34.0;  
  float map_y_max = 34.0;  

  int max_graph_size = 300;
  std::map<std::string, Node> graph;
  std::map<std::string, Node> graph_solution;
  Eigen::Vector2f sampled_point;
  float_t sampling_spread = 1.5;
  Node goal_node;
  bool goal_initialized = false;
  bool goal_found = false;
  float_t max_truncation_dist = 1.5;
  float_t min_truncation_dist = 0.01;
  float_t goal_loc_tolerance = 0.25;
  float_t goal_theta_tolerance = M_PI / 2.0;
  
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

  // Any time the Set Pose button is pushed
  if ((robot_loc_ - loc).norm() >= 0.1){
    // remove all the nodes from the graph and graph_solution
    ROS_INFO("Graph and solution cleared.");
    graph.clear();
    graph_solution.clear();

    // add the start_node to the graph
    Node start_node;
    std:: string id = to_string(loc.x()) + "," + to_string(loc.y()) + "," + to_string(angle);
    start_node.id = id;
    start_node.loc = loc;
    start_node.theta = angle;
    start_node.parent_id = "*";
    start_node.radius = 0.0;
    start_node.center_of_turn = Eigen::Vector2f(0.0,0.0);
    start_node.sampled_point = Eigen::Vector2f(0.0,0.0);
    start_node.theta_start = 0.0;
    start_node.theta_end = 0.0;
    start_node.path_length = 0.0;
    start_node.sp_theta = 0.0;
    start_node.sp_intersection = false;
    graph[id] = start_node;
    ROS_INFO("Start Node added to graph (%f,%f)", loc.x(), loc.y());
    
    // reset goal_node
    goal_node.id = "";
    goal_node.loc = Eigen::Vector2f(0.0,0.0);
    goal_node.theta = 0.0;
    goal_node.parent_id = "";
    goal_node.goal_theta_min = 0.0;
    goal_node.goal_theta_max = 0.0;
    goal_initialized = false;
    goal_found = false;
    ROS_INFO("Goal Node Reset.");
  }

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

  // Create ObjectAvoidance object that will manage path calculations
  path_planner_.reset(
    new object_avoidance::ObjectAvoidance(car_specs_, odd_num_paths, min_turn_radius_));
}

// Called anytime the Set Nav Target button is pushed
void Navigation::SetNavGoal(const Vector2f& loc, float angle) {

  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  // clear the previous solution
  graph_solution.clear();
  
  // update the goal_node information
  goal_node.id = to_string(loc.x()) + "," + to_string(loc.y()) + "," + to_string(angle);
  goal_node.loc = loc;
  goal_node.theta = angle;
  goal_node.parent_id = "";
  // float_t goal_theta_min = angle - goal_theta_tolerance;
  // if (goal_theta_min < - M_PI){
  //   goal_theta_min += 2.0 * M_PI;
  // }
  // float_t goal_theta_max = angle + goal_theta_tolerance;
  // if (goal_theta_max > M_PI){
  //   goal_theta_max -= 2.0 * M_PI;
  // }
  // goal_node.goal_theta_min = goal_theta_min;
  // goal_node.goal_theta_max = goal_theta_max;
  goal_node.goal_theta_min = angle - goal_theta_tolerance;
  goal_node.goal_theta_max = angle + goal_theta_tolerance;
  goal_initialized = true;

  ROS_INFO("Goal Target Added (%f, %f, %f)", loc.x(), loc.y(), angle);
}

// Return a Randomly Sampled point from the map
Eigen::Vector2f SamplePointFromMap() {
  float_t x_val;
  float_t y_val;
  float_t alpha;
  float_t randomf = rng_.UniformRandom(0.0, 1.0);

  if (randomf > 0.95){
    // "Exploration Bias" 5% of the time, greedily 
    //    attempt to get closer to the global graph
    alpha = rng_.UniformRandom(0.0, 2.0 * M_PI);
    x_val = goal_node.loc.x() + sampling_spread * cos(alpha);
    y_val = goal_node.loc.y() + sampling_spread * sin(alpha);
    // ROS_INFO("Sampling Goal Location = (%f, %f)", goal_node.loc.x(), goal_node.loc.y());
  } else if (randomf < 0.05) {
    // Randomly sample a point close to one of of the nodes
    // to encourage branching from "middle of the path" nodes
    auto node_ptr = graph.begin();
    int random_index = rng_.RandomInt(0, int(graph.size()));
    std::advance(node_ptr, random_index);
    alpha = rng_.UniformRandom(0.0, 2.0 * M_PI);

    x_val = node_ptr->second.loc.x() + sampling_spread * cos(alpha);
    y_val = node_ptr->second.loc.y() + sampling_spread * sin(alpha);
    // ROS_INFO("Sampling Node Location = (%f, %f)", node_ptr->second.loc.x(),node_ptr->second.loc.y());
  } else {
    // Sample point from map within map coordinates
    x_val = rng_.UniformRandom(map_x_min, map_x_max);
    y_val = rng_.UniformRandom(map_y_min, map_y_max);
    // ROS_INFO("Sampling Map Location.");
  }
  // ROS_INFO("Sampled Point = (%f, %f)", x_val, y_val);
  return Eigen::Vector2f(x_val, y_val);
}

// Draws a node as an arrow (location and heading)
void DrawNode(Node& node) {
  visualization::DrawArrow(node.loc, node.theta, 0x68ad7b, global_viz_msg_);
}

// Prints a nodes information
void PrintNode(Node& node){
  if (process_sample_debug) {
    ROS_INFO("id = %s",node.id.c_str());
    ROS_INFO("loc = (%f, %f)", node.loc.x(), node.loc.y());
    ROS_INFO("theta = %f", node.theta);
    ROS_INFO("parent_id = %s",node.parent_id.c_str());
    ROS_INFO("cot = (%f, %f)", node.center_of_turn.x(), node.center_of_turn.y());
    ROS_INFO("radius = %f", node.radius);
    ROS_INFO("theta_start = %f", node.theta_start);
    ROS_INFO("theta_end = %f", node.theta_end);
    ROS_INFO("path_length = %f", node.path_length);
    ROS_INFO("sampled_point = (%f, %f)", node.sampled_point.x(), node.sampled_point.y());
    ROS_INFO("------------------");
  }
}

// Draw the Target Goal
void Navigation::DrawTarget(bool& found) {
  uint32_t color;
  if (found){
    // green color if found
    color = 0x17c225;
  } else {
    // red color if not found
    color = 0xcc0c0c;
  }
  if (goal_node.loc != Eigen::Vector2f(0.0,0.0)){
    // Draws a circle around the target to easily identify target goal
    visualization::DrawArc(nav_goal_loc_, max_truncation_dist, 0.0, 2.0 * M_PI, color, global_viz_msg_);
    // Draws an arrow at the target goal (location and heading)
    visualization::DrawArrow(nav_goal_loc_, nav_goal_angle_, 0xcf25db, global_viz_msg_);
  }
}

// Draws the Graph of nodes
void DrawGraph(){
  ROS_INFO("=======================");
  ROS_INFO("Graph Contents: %ld items.", graph.size());
  std::map<std::string, Node>::iterator parent_id_ptr;

  for (auto const& x : graph) {
    Node node = x.second;
    PrintNode(node);
    DrawNode(node);

    parent_id_ptr = graph.find(node.parent_id);
    if (parent_id_ptr != graph.end()) {
      // Draws path between nodes
      visualization::DrawArc(node.center_of_turn, node.radius, node.theta_start, node.theta_end, 0x68ad7b, global_viz_msg_);
    }
  }
}

// Draws the solution to the found goal target as 
// a red line from the start node to the finish node
void DrawGraphSolution(){
  // ROS_INFO("=======================");
  // ROS_INFO("Graph Solution Contents: %ld items.", graph_solution.size());
  std::map<std::string, Node>::iterator parent_id_ptr;

  for (auto const& x : graph_solution) {
    Node node = x.second;
    // PrintNode(node);
    // DrawNode(node);
    parent_id_ptr = graph_solution.find(node.parent_id);
    if (parent_id_ptr != graph_solution.end()) {
      visualization::DrawArc(node.center_of_turn, node.radius, node.theta_start, node.theta_end, 0xcc0c0c, global_viz_msg_);
    }
  }
}

// Returns true if there is an intersection between any map line
// and the line from loc and point
bool Navigation::MapStraightLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point){
  int32_t num_map_lines = (int32_t)map_.lines.size();
  geometry::line2f my_line(loc, point);
  for (int32_t j = 0; j < num_map_lines; j++) {
    if (map_.lines[j].Intersects(my_line)){
      return true;
    }
  }
  return false;
}

// Returns true if there is an intersection between any map line
// and the curved line from loc to point
bool Navigation::MapCurvedLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point){
  return false;
}

// Called after goal is found: adds nodes from graph
// to graph_solution
void Navigation::GenerateGraphSolution(){
  std::string node_id = goal_node.parent_id;
  Node new_node; 

  while (node_id != "*"){
    graph_solution[node_id] = graph[node_id];
    node_id = graph[node_id].parent_id;
  }

}

// Iterates over all nodes as candidate parents to the 
// final goal location and heading. Admits the final node
// if it is within a location/heading tolerance. If such a 
// node exists, adds the appropriate nodes to the graph solution 
void Navigation::FindPathToGoal() {
  Eigen::Vector2f goal_loc = goal_node.loc;
  // float_t goal_theta = goal_node.theta;

  Node new_node;

  for (auto const& node_itr : graph) {
    std::string candidate_parent_id = node_itr.second.id;
    new_node = ProcessSampledPointFromParent(goal_loc, candidate_parent_id);

    // if node parent_id is not blank
    if (new_node.parent_id != "" &&
          // and node final heading is within heading tolerance
          ((new_node.theta <= goal_node.goal_theta_max &&
              new_node.theta >= goal_node.goal_theta_min) ||
           (new_node.theta + 2.0 * M_PI <= goal_node.goal_theta_max &&
              new_node.theta + 2.0 * M_PI >= goal_node.goal_theta_min) ||
           (new_node.theta - 2.0 * M_PI <= goal_node.goal_theta_max &&
              new_node.theta - 2.0 * M_PI >= goal_node.goal_theta_min)) &&
                // and node location is within location tolerance
                (new_node.loc - goal_loc).norm() <= goal_loc_tolerance) {
      graph[new_node.id] = new_node;
      goal_node.parent_id = new_node.id;
      ROS_INFO("Goal found!");
      GenerateGraphSolution();
      return;
    }

    // // a truncated point did not get close to goal, but maybe it would
    // // have if it wasn't truncated.
    // // if node parent_id is not blank
    // if (new_node.parent_id != "" &&
    //       !new_node.sp_intersection &&
    //         // and node heading at sampled point is within heading tolerance
    //         ((new_node.sp_theta <= goal_node.goal_theta_max &&
    //             new_node.sp_theta >= goal_node.goal_theta_min) ||
    //         (new_node.sp_theta + 2.0 * M_PI <= goal_node.goal_theta_max &&
    //             new_node.sp_theta + 2.0 * M_PI >= goal_node.goal_theta_min) ||
    //         (new_node.sp_theta - 2.0 * M_PI <= goal_node.goal_theta_max &&
    //             new_node.sp_theta - 2.0 * M_PI >= goal_node.goal_theta_min))) {
    //   graph[new_node.id] = new_node;
    // }


    // // if node parent_id is not blank
    // if (new_node.parent_id != "" &&
    //       // and node heading is within heading tolerance
    //       (new_node.theta <= goal_theta + goal_theta_tolerance &&
    //           new_node.theta >= goal_theta - goal_theta_tolerance) &&
    //             // and node location is within location tolerance
    //             (new_node.loc - goal_loc).norm() <= goal_loc_tolerance) {
    //   graph[new_node.id] = new_node;
    //   goal_node.parent_id = new_node.id;
    //   ROS_INFO("Goal found!");
    //   GenerateGraphSolution();
    //   return;
    // }
  }
}

// Returns the truncated point assuming a straight line path from 
// the loc to point at a max_dist away from loc (used for previous
// simpler implementation)
Eigen::Vector2f CalculateStraightTruncation(Eigen::Vector2f& loc, Eigen::Vector2f& point, float_t max_dist){
  float_t dist = (loc - point).norm();
  return Eigen::Vector2f(loc.x() + max_dist * (point.x() - loc.x()) / dist, loc.y() + max_dist * (point.y() - loc.y()) / dist);
}

// Returns a node representing the final location/heading along a constant
// curvature arc from the parent node to the sample_point at a max distance 
// from the parent node
Node Navigation::ProcessSampledPointFromParent(Eigen::Vector2f& sample_point, std::string parent_id){
  Node new_node;
  Eigen::Vector2f truncated_point;
  new_node.parent_id = parent_id;
  
  Eigen::Vector2f parent_loc = graph[parent_id].loc;
  float_t parent_theta = graph[parent_id].theta;
  
  float_t x_r = parent_loc.x();
  float_t y_r = parent_loc.y();
  float_t theta_r = parent_theta;
  float_t L_max = max_truncation_dist;
  float_t x_s = sample_point.x();
  float_t y_s = sample_point.y();

  float_t x_h = x_r + cos(theta_r);
  float_t y_h = y_r + sin(theta_r);
  float_t m_hr = (y_h - y_r)/(x_h - x_r);
  float_t b_hr = y_r - m_hr * x_r;

  float_t m_T;
  if (abs(y_h - y_r) == 0) {
    m_T = -(x_h - x_r)/(y_h - y_r + 1e-6); // don't divide by zero
  } else {
    m_T = -(x_h - x_r)/(y_h - y_r);
  }
  float_t b_T = y_r - m_T * x_r;

  float_t x_T = (-2.0 * b_T * y_r + 2.0 * b_T * y_s + pow(x_r,2.0) + pow(y_r,2.0) - pow(y_s,2.0) - pow(x_s,2.0)) / (2.0 * x_r + 2 * y_r * m_T - 2.0 * y_s * m_T - 2.0 * x_s);
  float_t y_T = m_T * x_T + b_T;

  float_t R = sqrt(pow(x_T - x_r, 2.0) + pow(y_T - y_r, 2.0));

  float_t s;
  float_t s_0 = -(y_s - m_hr * x_s - b_hr) / abs(y_s - m_hr * x_s - b_hr);  
  if (abs(theta_r) <= M_PI/2.0) {
    s = s_0;
  } else {
    s = -s_0;
  }

  float_t theta_tan = atan2(s * (x_T - x_s), s * (y_s - y_T));

  float_t theta_d;
  float_t theta_d_0 = theta_r - theta_tan;
  if (abs(theta_d_0) < M_PI) {
    theta_d = theta_d_0;
  } else {
    theta_d = theta_d_0 - ((theta_d_0) / abs(theta_d_0)) * 2.0 * M_PI;
  }

  float_t L = s * R * theta_d;
  float_t theta_max = s * min(abs(L_max / R), abs(theta_d));
  float_t d_max = sqrt(2.0 * pow(R,2.0) * (1 - cos(theta_max)));
  float_t L_tp = s * R * theta_max;

  float_t B_f = ((M_PI - theta_max) / 2.0) - (M_PI / 2.0 - theta_r);
  float_t B_b = (M_PI - theta_max) - (B_f + (M_PI / 2.0));

  float_t x_max;
  float_t y_max;
  if (L >= 0.0) {
    x_max = x_r + d_max * cos(B_f);
    y_max = y_r + d_max * sin(B_f);
  } else {
    x_max = x_r - d_max * sin(B_b);
    y_max = y_r - d_max * cos(B_b);
  }

  float_t theta_maxtan = atan2(s * (x_T - x_max), s * (y_max - y_T));

  float_t theta_s;
  float_t theta_s_0 = theta_r + s * M_PI / 2.0;
  if (abs(theta_s_0) < M_PI){
    theta_s = theta_s_0;
  } else {
    theta_s = theta_s_0 - (theta_s_0 / abs(theta_s_0)) * 2.0 * M_PI;
  }

  float_t theta_e;
  float_t theta_e_0 = theta_maxtan + s * M_PI / 2.0;
  if (abs(theta_e_0) < M_PI){
    theta_e = theta_e_0;
  } else {
    theta_e = theta_e_0 - (theta_e_0 / abs(theta_e_0)) * 2.0 * M_PI;
  }

  truncated_point = Eigen::Vector2f(x_max,y_max);
  if (process_sample_debug){
    ROS_INFO("---------------------");
    ROS_INFO("x_r = %f", x_r);
    ROS_INFO("y_r = %f", y_r);
    ROS_INFO("theta_r = %f", theta_r);
    ROS_INFO("L_max = %f", L_max);
    ROS_INFO("x_s = %f", x_s);
    ROS_INFO("y_s = %f", y_s);

    ROS_INFO("x_h = %f", x_h);
    ROS_INFO("y_h = %f", y_h);
    ROS_INFO("m_hr = %f", m_hr);
    ROS_INFO("b_hr = %f", b_hr);

    ROS_INFO("m_T = %f", m_T);
    ROS_INFO("b_T = %f", b_T);

    ROS_INFO("x_T = %f", x_T);
    ROS_INFO("y_T = %f", y_T);

    ROS_INFO("R = %f", R);

    ROS_INFO("s_0 = %f", s_0);
    ROS_INFO("s = %f", s);

    ROS_INFO("theta_tan = %f", theta_tan);

    ROS_INFO("theta_d_0 = %f", theta_d_0);
    ROS_INFO("theta_d = %f", theta_d);

    ROS_INFO("L = %f", L);
    ROS_INFO("theta_max = %f", theta_max);
    ROS_INFO("d_max = %f", d_max);
    ROS_INFO("L_tp = %f", L_tp);

    ROS_INFO("B_f = %f", B_f);
    ROS_INFO("B_b = %f", B_b);

    ROS_INFO("x_max = %f", x_max);
    ROS_INFO("y_max = %f", y_max);

    ROS_INFO("theta_maxtan = %f", theta_maxtan);

    ROS_INFO("theta_s_0 = %f", theta_s_0);
    ROS_INFO("theta_s = %f", theta_s);

    ROS_INFO("theta_e_0 = %f", theta_e_0);
    ROS_INFO("theta_e = %f", theta_e);
    ROS_INFO("-------------------");

    ROS_INFO("Truncated Point = (%f, %f)", truncated_point.x(), truncated_point.y());
  }

  // if calculated turn radius is too tight
  if ((R < min_turn_radius_) || 
          // or calculated path has an intersection with the map
          (MapStraightLineIntersection(parent_loc, truncated_point)) || 
              // or truncated point is virutally the same as the parent node
              (parent_loc - truncated_point).norm() < min_truncation_dist) { 
    // this stops node from being added to graph
    new_node.parent_id = "";
  }
  
  std::string id = to_string(x_max) + "," + to_string(y_max) + "," + to_string(theta_maxtan);
  new_node.id = id;
  new_node.loc = truncated_point;
  new_node.theta = theta_maxtan;
  new_node.radius = R;
  new_node.center_of_turn = Eigen::Vector2f(x_T,y_T);
  new_node.path_length = L_tp;
  new_node.sampled_point = sample_point;
  new_node.sp_theta = theta_tan;
  new_node.sp_intersection = MapStraightLineIntersection(parent_loc, sample_point);

  // These conditionals ensure the arc gets drawn in the correct 
  // direction depending on the orientation of several items 
  // such as theta_s, L, and theta_r
  if (theta_s > 0){
    if (L < 0) {
      if (abs(theta_r) > M_PI / 2.0){
        new_node.theta_start = theta_e;
        new_node.theta_end = theta_s;
      } else {
        new_node.theta_start = theta_s;
        new_node.theta_end = theta_e;
      }
    } else {
      if (abs(theta_r) > M_PI / 2.0){
        new_node.theta_start = theta_s;
        new_node.theta_end = theta_e;
      } else {
        new_node.theta_start = theta_e;
        new_node.theta_end = theta_s;
      }
    }
  } else {
    if (L < 0){
      if (abs(theta_r) > M_PI / 2.0){
        new_node.theta_start = theta_s;
        new_node.theta_end = theta_e;
      } else {
        new_node.theta_start = theta_e;
        new_node.theta_end = theta_s;
      }
    } else{
      if (abs(theta_r) > M_PI / 2.0){
        new_node.theta_start = theta_e;
        new_node.theta_end = theta_s;
      } else {
        new_node.theta_start = theta_s;
        new_node.theta_end = theta_e;
      }
    }
  }
  return new_node;
}

// Given a sample point, return the candidate node based on which 
// parent node is closest to the sample point
Node Navigation::ProcessSampledPointClosestNode(Eigen::Vector2f& sample_point){
  Node new_node;
  std::string candidate_parent_id = "";

  float_t min_dist = 150.0;
  for (auto const& node_itr : graph) {
    float_t dist = (node_itr.second.loc - sample_point).norm();
    if (dist < min_dist){
      min_dist = dist;
      candidate_parent_id = node_itr.second.id;
    }
  }
  
  new_node = ProcessSampledPointFromParent(sample_point, candidate_parent_id);
  return new_node;
}

// Given a sample point, return the candidate node based on which 
// gives the furthest free path from any of the parent nodes
Node Navigation::ProcessSampledPointFurthestPath(Eigen::Vector2f& sample_point){
  Node new_node;
  Node best_node;
  std::string candidate_parent_id = "";

  float_t max_path_length = 0.0;
  for (auto const& node_itr : graph) {
    candidate_parent_id = node_itr.second.id;
    new_node = ProcessSampledPointFromParent(sample_point, candidate_parent_id);

    if (new_node.parent_id != "" && new_node.path_length > max_path_length){
      max_path_length = new_node.path_length;
      best_node = new_node;
    }
  }
  
  return best_node;
}

void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  
  // goal is found when the goal_node parent_id is not blank
  goal_found = (goal_node.parent_id != "");

  drive_msg_.velocity = 0.0;
  // Draw the starting location as a car
  DrawCar();
  // Draw the target location
  DrawTarget(goal_found);
  // Draw the nodes in the graph and the curved edges between them
  DrawGraph();
  // Draw the Graph Solution when found as a red line between nodes
  DrawGraphSolution();

  // if the goal hasn't been found yet
  if (goal_initialized && !goal_found) {

    // randomly sample a point from the map
    sampled_point = SamplePointFromMap();
    Node new_node;
    // create a new node which is kinematically feasible at
    // a maximum distance from the closest node
    new_node = ProcessSampledPointClosestNode(sampled_point);

    // only admit the node if the parent id is not blank
    if (new_node.parent_id != ""){
      graph[new_node.id] = new_node;
    }
    
    // attempt to find a kinematic solution to the goal
    // and update goal_node.parent_id if one is found
    FindPathToGoal();
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  // limit the number of nodes that are added to the graph
  // before exiting the program
  if (int(graph.size()) > max_graph_size){
    ROS_INFO("%d points added to graph.", max_graph_size);
    exit(3);
  }

}

}  // namespace navigation
