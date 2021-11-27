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
// #include "node.h"

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

  float map_x_min = -44.0;
  float map_x_max = 44.0; 
  float map_y_min = -34.0;  
  float map_y_max = 34.0;  

  // struct Node {
  //   std::string id;
  //   Eigen::Vector2f loc;
  //   float_t theta;
  //   std::string parent_id;
  //   Eigen::Vector2f turn_point_to_node;
  // };

  std::map<std::string, Node> graph;
  // std::vector<Eigen::Vector2f> sampled_points;
  // int num_sampled_points = 50;
  Eigen::Vector2f sampled_point;
  Node goal_node;
  bool goal_initialized = false;
  bool goal_found = false;
  float_t max_truncation_dist = 2.0;
  
  // Eigen::Vector2f sampled_point;
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

  // Any time the Set Pose is pushed.
  if ((robot_loc_ - loc).norm() >= 0.1){
    // remove all the nodes from the graph
    ROS_INFO("Graph cleared.");
    graph.clear();

    // add the start_node to the graph
    Node start_node;
    std:: string id = to_string(loc.x()) + "," + to_string(loc.y()) + "," + to_string(angle);
    start_node.id = id;
    start_node.loc = loc;
    start_node.theta = angle;
    start_node.parent_id = "*";
    graph[id] = start_node;
    ROS_INFO("Start Node added to graph (%f,%f)", loc.x(), loc.y());
    
    // reset goal_node
    goal_node.id = "";
    goal_node.loc = Eigen::Vector2f(0.0,0.0);
    goal_node.theta = 0.0;
    goal_node.parent_id = "";
    goal_initialized = false;
    goal_found = false;
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

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;

  goal_initialized = true;
  
  goal_node.id = to_string(loc.x()) + "," + to_string(loc.y()) + "," + to_string(angle);
  goal_node.loc = loc;
  goal_node.theta = angle;
  goal_node.parent_id = "";
  
  ROS_INFO("Goal Target Added (%f, %f, %f)", loc.x(), loc.y(), angle);
}

Eigen::Vector2f SamplePointFromMap() {
  float_t x_val;
  float_t y_val;
  if (rng_.UniformRandom(0.0, 1.0) > 0.95){
    // "Exploration Bias" 5% of the time, greedily 
    //    attempt to get closer to the global graph
    x_val = goal_node.loc.x();
    y_val = goal_node.loc.y();
  } else {
    x_val = rng_.UniformRandom(map_x_min, map_x_max);
    y_val = rng_.UniformRandom(map_y_min, map_y_max);
  }
  ROS_INFO("Sampled Point = (%f, %f)", x_val, y_val);
  return Eigen::Vector2f(x_val, y_val);
}

void DrawNode(Node& node) {
  visualization::DrawArrow(node.loc, node.theta, 0x68ad7b, global_viz_msg_);
}

void PrintNode(Node& node){
  ROS_INFO("id = %s",node.id.c_str());
  ROS_INFO("loc = (%f, %f)", node.loc.x(), node.loc.y());
  ROS_INFO("theta = %f", node.theta);
  ROS_INFO("parent_id = %s",node.parent_id.c_str());
  ROS_INFO("cot = (%f, %f)", node.center_of_turn.x(), node.center_of_turn.y());
  ROS_INFO("radius = %f", node.radius);
  ROS_INFO("theta_start = %f", node.theta_start);
  ROS_INFO("theta_end = %f", node.theta_end);
  ROS_INFO("------------------");
}

void Navigation::DrawTarget(bool& found) {
  uint32_t color;
  if (found){
    color = 0x17c225;
  } else {
    color = 0xcc0c0c;
  }
  if (goal_node.loc != Eigen::Vector2f(0.0,0.0)){
    ROS_INFO("Goal Location = (%f, %f)", goal_node.loc.x(), goal_node.loc.y());
    visualization::DrawArc(nav_goal_loc_, max_truncation_dist, 0.0, 2.0 * M_PI, color, global_viz_msg_);
    visualization::DrawArrow(nav_goal_loc_, nav_goal_angle_, 0xcc0c0c, global_viz_msg_);
  }
}

Eigen::Vector2f GetLocation(std::string& id){
  return graph[id].loc;
}

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
      visualization::DrawArc(node.center_of_turn, node.radius, node.theta_start, node.theta_end, 0x68ad7b, global_viz_msg_);
      // visualization::DrawArc(node.center_of_turn, 0.1, 0, 2.0 * M_PI, 0x68ad7b, global_viz_msg_);
      // Straight line from node to node
      // visualization::DrawLine(node.loc, parent_id_ptr->second.loc,0x68ad7b,global_viz_msg_);
    }
  }
}

bool Navigation::MapStraightLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point){
  int32_t num_map_lines = (int32_t)map_.lines.size();
  geometry::line2f my_line(loc, point);
  for (int32_t j = 0; j < num_map_lines; j++) {
    if (map_.lines[j].Intersects(my_line)){
      return true;
    }
  }
  ROS_INFO("No intersection found between (%f,%f) (%f,%f)", loc.x(), loc.y(), point.x(), point.y());
  return false;
}

bool Navigation::MapCurvedLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point){
  return false;
}

void Navigation::FindPathToGoal() {
  Node closest_node;
  Eigen::Vector2f goal_loc = goal_node.loc;

  float_t min_dist = 150.0;
  for (auto const& node_itr : graph) {
    float_t d = (node_itr.second.loc - goal_loc).norm();
    if (d < min_dist){
      min_dist = d;
      closest_node = graph[node_itr.second.id];
    }
  }

  ROS_INFO("Distance To Goal = %f", min_dist);
  if (!MapStraightLineIntersection(closest_node.loc, goal_loc) &&
          min_dist <= max_truncation_dist) {
    goal_node.parent_id = closest_node.id;
  }

}

Eigen::Vector2f CalculateStraightTruncation(Eigen::Vector2f& loc, Eigen::Vector2f& point, float_t max_dist){
  float_t dist = (loc - point).norm();
  return Eigen::Vector2f(loc.x() + max_dist * (point.x() - loc.x()) / dist, loc.y() + max_dist * (point.y() - loc.y()) / dist);
}

Node Navigation::ProcessSampledPoint(Eigen::Vector2f& sample_point){
  Node best_new_node;
  best_new_node.parent_id = "";

  Eigen::Vector2f truncated_point;

  float_t best_distance = 0.0;
  for (auto const& node_itr : graph) {

    std::string candidate_parent_id = node_itr.second.id;    
    Eigen::Vector2f parent_loc = graph[candidate_parent_id].loc;
    float_t parent_theta = graph[candidate_parent_id].theta;
    
    //-----BEGIN COMPLICATED SECTION FOR CURVES
    float_t x_r = parent_loc.x();
    float_t y_r = parent_loc.y();
    float_t theta_r = parent_theta;
    float_t L_max = max_truncation_dist;
    float_t x_s = sample_point.x();
    float_t y_s = sample_point.y();
    ROS_INFO("---------------------");
    ROS_INFO("x_r = %f", x_r);
    ROS_INFO("y_r = %f", y_r);
    ROS_INFO("theta_r = %f", theta_r);
    ROS_INFO("L_max = %f", L_max);
    ROS_INFO("x_s = %f", x_s);
    ROS_INFO("y_s = %f", y_s);

    float_t x_h = x_r + cos(theta_r);
    float_t y_h = y_r + sin(theta_r);
    float_t m_hr = (y_h - y_r)/(x_h - x_r);
    float_t b_hr = y_r - m_hr * x_r;
    ROS_INFO("x_h = %f", x_h);
    ROS_INFO("y_h = %f", y_h);
    ROS_INFO("m_hr = %f", m_hr);
    ROS_INFO("b_hr = %f", b_hr);

    float_t m_T;
    if (abs(y_h - y_r) == 0) {
      m_T = -(x_h - x_r)/(y_h - y_r + 1e-6); // don't divide by zero
    } else {
      m_T = -(x_h - x_r)/(y_h - y_r);
    }
    float_t b_T = y_r - m_T * x_r;
    ROS_INFO("m_T = %f", m_T);
    ROS_INFO("b_T = %f", b_T);

    float_t x_T = (-2.0 * b_T * y_r + 2.0 * b_T * y_s + pow(x_r,2.0) + pow(y_r,2.0) - pow(y_s,2.0) - pow(x_s,2.0)) / (2.0 * x_r + 2 * y_r * m_T - 2.0 * y_s * m_T - 2.0 * x_s);
    float_t y_T = m_T * x_T + b_T;
    ROS_INFO("x_T = %f", x_T);
    ROS_INFO("y_T = %f", y_T);

    float_t R = sqrt(pow(x_T - x_r, 2.0) + pow(y_T - y_r, 2.0));
    ROS_INFO("R = %f", R);

    float_t s;
    float_t s_0 = -(y_s - m_hr * x_s - b_hr) / abs(y_s - m_hr * x_s - b_hr);  
    if (abs(theta_r) <= M_PI/2.0) {
      s = s_0;
    } else {
      s = -s_0;
    }
    ROS_INFO("s_0 = %f", s_0);
    ROS_INFO("s = %f", s);

    float_t theta_tan = atan2(s * (x_T - x_s), s * (y_s - y_T));
    ROS_INFO("theta_tan = %f", theta_tan);

    float_t theta_d;
    float_t theta_d_0 = theta_r - theta_tan;
    if (abs(theta_d_0) < M_PI) {
      theta_d = theta_d_0;
    } else {
      theta_d = theta_d_0 - ((theta_d_0) / abs(theta_d_0)) * 2.0 * M_PI;
    }
    ROS_INFO("theta_d_0 = %f", theta_d_0);
    ROS_INFO("theta_d = %f", theta_d);

    float_t L = s * R * theta_d;
    float_t theta_max = s * min(abs(L_max / R), abs(theta_d));
    float_t d_max = sqrt(2.0 * pow(R,2.0) * (1 - cos(theta_max)));
    float_t L_tp = s * R * theta_max;
    ROS_INFO("L = %f", L);
    ROS_INFO("theta_max = %f", theta_max);
    ROS_INFO("d_max = %f", d_max);
    ROS_INFO("L_tp = %f", L_tp);

    float_t B_f = ((M_PI - theta_max) / 2.0) - (M_PI / 2.0 - theta_r);
    float_t B_b = (M_PI - theta_max) - (B_f + (M_PI / 2.0));
    ROS_INFO("B_f = %f", B_f);
    ROS_INFO("B_b = %f", B_b);

    float_t x_max;
    float_t y_max;
    if (L >= 0.0) {
      x_max = x_r + d_max * cos(B_f);
      y_max = y_r + d_max * sin(B_f);
    } else {
      x_max = x_r - d_max * sin(B_b);
      y_max = y_r - d_max * cos(B_b);
    }
    ROS_INFO("x_max = %f", x_max);
    ROS_INFO("y_max = %f", y_max);

    float_t theta_maxtan = atan2(s * (x_T - x_max), s * (y_max - y_T));
    ROS_INFO("theta_maxtan = %f", theta_maxtan);

    float_t theta_s;
    float_t theta_s_0 = theta_r + s * M_PI / 2.0;
    if (abs(theta_s_0) < M_PI){
      theta_s = theta_s_0;
    } else {
      theta_s = theta_s_0 - (theta_s_0 / abs(theta_s_0)) * 2.0 * M_PI;
    }
    ROS_INFO("theta_s_0 = %f", theta_s_0);
    ROS_INFO("theta_s = %f", theta_s);

    float_t theta_e;
    float_t theta_e_0 = theta_maxtan + s * M_PI / 2.0;
    if (abs(theta_e_0) < M_PI){
      theta_e = theta_e_0;
    } else {
      theta_e = theta_e_0 - (theta_e_0 / abs(theta_e_0)) * 2.0 * M_PI;
    }
    ROS_INFO("theta_e_0 = %f", theta_e_0);
    ROS_INFO("theta_e = %f", theta_e);
    ROS_INFO("-------------------");

    //-----END COMPLICATED SECTION FOR CURVES

    truncated_point = Eigen::Vector2f(x_max,y_max);
    ROS_INFO("Truncated Point = (%f, %f)", truncated_point.x(), truncated_point.y());
    
    if ((R < min_turn_radius_) || // calculated turn radius is too tight
            (MapStraightLineIntersection(parent_loc, truncated_point))) { // calculated path has an intersection with the map
      // this stops node from being added to graph
      candidate_parent_id = "";
    }
    
    if (L_tp > best_distance 
            && candidate_parent_id != ""){
      best_distance = L_tp; 

      // ensure the arc gets drawn in the correct direction depending 
      // on the orientation of several items such as s, L, and theta_r
      if (theta_s > 0){
        if (L < 0) {
          if (abs(theta_r) > M_PI / 2.0){
            best_new_node.theta_start = theta_e;
            best_new_node.theta_end = theta_s;
          } else {
            best_new_node.theta_start = theta_s;
            best_new_node.theta_end = theta_e;
          }
        } else {
          if (abs(theta_r) > M_PI / 2.0){
            best_new_node.theta_start = theta_s;
            best_new_node.theta_end = theta_e;
          } else {
            best_new_node.theta_start = theta_e;
            best_new_node.theta_end = theta_s;
          }
        }
      } else {
        if (L < 0){
          if (abs(theta_r) > M_PI / 2.0){
            best_new_node.theta_start = theta_s;
            best_new_node.theta_end = theta_e;
          } else {
            best_new_node.theta_start = theta_e;
            best_new_node.theta_end = theta_s;
          }
        } else{
          if (abs(theta_r) > M_PI / 2.0){
            best_new_node.theta_start = theta_e;
            best_new_node.theta_end = theta_s;
          } else {
            best_new_node.theta_start = theta_s;
            best_new_node.theta_end = theta_e;
          }
        }
      }

    best_new_node.id = to_string(x_max) + "," + to_string(y_max) + "," + to_string(theta_maxtan);
    best_new_node.loc = truncated_point;
    best_new_node.theta = theta_maxtan;
    best_new_node.radius = R;
    best_new_node.center_of_turn = Eigen::Vector2f(x_T, y_T);;
    best_new_node.parent_id = candidate_parent_id;
    } // end if - keep track of best candidate parent/truncation_point
    
  } // end loop - find best candidate parent/truncation_point
    
  return best_new_node;
}


void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  
  goal_found = (goal_node.parent_id != "");

  drive_msg_.velocity = 0.0;
  DrawCar();
  DrawTarget(goal_found);
  DrawGraph();

  if (goal_initialized && !goal_found) {

    sampled_point = SamplePointFromMap();
    Node new_node;
    new_node = ProcessSampledPoint(sampled_point);

    if (new_node.parent_id != ""){
      graph[new_node.id] = new_node;
    }
    
    // FindPathToGoal();
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  if (graph.size() > 100){
    exit(3);
  }

}

}  // namespace navigation
