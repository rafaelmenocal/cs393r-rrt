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

#include <cstdlib>
#include <cmath>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "math/line2d.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "graph.h"
#include "global_planner.h"

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

float map_x_width = 100.0; // -50.0 to +50.0
float map_y_height = 100.0;  // -50.0 to +50.0
float resolution = 0.25; // meters between nodes
float local_target_radius = 2.0; // meters away from robot

Eigen::MatrixXf occupancy_matrix = Eigen::MatrixXf::Ones(1 + int(map_x_width/resolution), 1 + int(map_y_height/resolution));
std::map<std::string, Node*> Nodes;
std::vector<std::string> global_path;

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
  visualization::DrawRobot(car_width_, car_length_, rear_axle_offset_, car_safety_margin_front_, car_safety_margin_side_, drive_msg_, local_viz_msg_, collision);
  // visualization::DrawLocalTarget(local_target, local_viz_msg_);
  visualization::DrawPathOption(drive_msg_.curvature, local_target_radius, 0, local_viz_msg_);
  
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



// -------- BEGIN CODE FOR NAVIGATION PLANNER -----------------

// given the x and y map coordinates, return the closest index i, j
Eigen::Vector2i PointToIndex(Eigen::Vector2f point){
  return Eigen::Vector2i(int(round(((map_y_height/2.0) - point.y())/resolution)), int(round((point.x() + (map_x_width/2))/resolution)));
}

// given the matrix indices, return the map x and y location
Eigen::Vector2f IndexToPoint(Eigen::Vector2i loc){
  return Eigen::Vector2f((loc.y() * resolution) - (map_x_width / 2), (map_y_height / 2) - (loc.x() * resolution));
}

// given the robot location, highlight nearest node in red
void DrawRobotNode(Eigen::Vector2f loc){
  Eigen::Vector2f point =IndexToPoint(PointToIndex(loc));
  visualization::DrawArc(point, 0.3, 0, 2.0 * M_PI, 0x0045cf, global_viz_msg_);
  visualization::DrawPoint(point, 0x0045cf, global_viz_msg_);
}

// given the target location, highlight nearest node in green
void DrawTargetNode(Eigen::Vector2f loc){
  Eigen::Vector2f point =IndexToPoint(PointToIndex(loc));
  visualization::DrawArc(point, 0.3, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
  visualization::DrawPoint(point, 0x3ede12, global_viz_msg_);
}

// Iterate through occupancy_matrix to overlay the nodes on the map
// Note: This is computationally expensive to draw so it should only 
// be used for debugging.
void DrawOccupancyMatrix() {
  for (int i = 0; i < occupancy_matrix.cols(); i+=1){
    for (int j = 0; j < occupancy_matrix.rows(); j+=1){
      if (occupancy_matrix(i, j) == 1){
        Eigen::Vector2f point = IndexToPoint(Eigen::Vector2i(i, j));
        visualization::DrawPoint(point, 0x3ede12, global_viz_msg_);
      }
    }
  }
}

// Highlights the nodes associated with the walls in the map
void DrawMapPoints(vector_map::VectorMap map){
  int32_t num_map_lines = (int32_t)map.lines.size();
  for (int32_t j = 0; j < num_map_lines; j++) {
    ROS_INFO("processing line %d", j);
    geometry::line2f line = map.lines[j];

    float x1 = line.p0(0,0);
    float y1 = line.p0(1,0);
    float x2 = line.p1(0,0);
    float y2 = line.p1(1,0);

    float length = (Eigen::Vector2f(x1,y1) - Eigen::Vector2f(x2,y2)).norm();
    int num_points = 2 * int(length / resolution);
    float x_inc = (x2 - x1) / num_points;
    float y_inc = (y2 - y1) / num_points;

    for (int i = 0; i < num_points; i++){
      Eigen::Vector2f point = Eigen::Vector2f(x1,y1) + Eigen::Vector2f(i * x_inc, i * y_inc);
      visualization::DrawArc(point, 0.05, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_);
    }
  visualization::DrawArc(Eigen::Vector2f(x2, y2), 0.05, 0, 2.0 * M_PI, 0xd60b26, global_viz_msg_); 
  }
}

void UpdateGlobalGraph (geometry::line2f line) {
  float x1 = line.p0(0,0);
  float y1 = line.p0(1,0);
  float x2 = line.p1(0,0);
  float y2 = line.p1(1,0);

  float length = (Eigen::Vector2f(x1,y1) - Eigen::Vector2f(x2,y2)).norm();
  int num_points = 2 * int(length / resolution);
  float x_inc = (x2 - x1) / num_points;
  float y_inc = (y2 - y1) / num_points;

  for (int i = 0; i < num_points; i++){
    Eigen::Vector2f point = Eigen::Vector2f(x1,y1) + Eigen::Vector2f(i * x_inc, i * y_inc);
    Eigen::Vector2i index = PointToIndex(point);
    occupancy_matrix(index.x(), index.y()) = 0;
  }
  Eigen::Vector2i index = PointToIndex(Eigen::Vector2f(x2, y2));
  occupancy_matrix(index.x(), index.y()) = 0;
}

void DrawNeighbors (Node* node){
  ROS_INFO("node id = %s", node->id.c_str());
  visualization::DrawArc(Eigen::Vector2f(node->x, node->y), 0.1, 0, 2.0 * M_PI, 0xeb34d2, global_viz_msg_);
  for (std::string n : node->neighbor_ids){
    ROS_INFO("  neighbor = %s", n.c_str());
    Node* node2 = Nodes[n];
    visualization::DrawArc(Eigen::Vector2f(node2->x, node2->y), 0.1, 0, 2.0 * M_PI, 0xeba534, global_viz_msg_);
  } 
}

void InitializeOccupancyMatrix(vector_map::VectorMap map) {
  
  // occupancy_matrix initialized to all ones prior to this
  int32_t num_map_lines = (int32_t)map.lines.size();

  // iterate through all the map lines in the graph
  for (int32_t j = 0; j < num_map_lines; j++) {
    ROS_INFO("processing line %d", j);
    // set any cell in occupancy_matrix to 0 if map line intersects that cell
    UpdateGlobalGraph(map.lines[j]);
  }  


  Eigen::MatrixXf occupancy_matrix_copy = occupancy_matrix;
  //loop through i, j in global graph and make entry
  // 0 if bordered by a 0 eg. "pads the walls"
  for (int i = 0; i < occupancy_matrix.rows(); i++){
    for (int j = 0; j < occupancy_matrix.cols(); j++){
      for (int a = i - 1; a <= i + 1; a++){
          for (int b = j - 1; b <= j + 1; b++){
            if (((a != i) || (b != j))
                  && ((a >= 0) && (a < occupancy_matrix.rows())) // valid first dimension
                  && ((b >= 0) && (b < occupancy_matrix.cols()))){
              if (occupancy_matrix(a,b) == 0){ // bordered by invalid node
                occupancy_matrix_copy(i,j) = 0;
              }
            }
          }
      }
    }
  }
  occupancy_matrix = occupancy_matrix_copy;

  // loops through every i, j in occupancy_matrix
  for (int i = 0; i < occupancy_matrix.rows(); i++){
    for (int j = 0; j < occupancy_matrix.cols(); j++){
      
      if (occupancy_matrix(i,j)==1){
        string id = std::to_string(i) + "," + std::to_string(j);
        Eigen::Vector2f point = IndexToPoint(Eigen::Vector2i(i,j));
        std::vector<std::string> neighbor_ids;
        
        Node* n = new Node();
        n->id = id;
        n->x = point.x();
        n->y = point.y();

        // loops through the adjacent 9 nodes
        for (int a = i - 1; a <= i + 1; a++){
          for (int b = j - 1; b <= j + 1; b++){
            if (((a != i) || (b != j)) // node can't be its own neighbor
                  && ((a >= 0) && (a < occupancy_matrix.rows())) // valid first dimension
                  && ((b >= 0) && (b < occupancy_matrix.cols()))){ // valid second column
                
                if (occupancy_matrix(a,b) == 1){ // a valid neigbor exists
                  string n_id = std::to_string(a) + "," + std::to_string(b);
                  neighbor_ids.push_back(n_id);
                }
            }
          }
        }
        n->neighbor_ids = neighbor_ids;
        Nodes[id] = n;
      }
    }
  }
  ROS_INFO("Nodes.size() = %ld", Nodes.size());
  ROS_INFO("Initialization complete.");
}

void DrawGlobalPath() {
  Node* prev_node;
  bool first_point = true;
  for (std::string p : global_path){
    Node* node = Nodes[p];
    if (!first_point) {
      visualization::DrawLine(Eigen::Vector2f(prev_node->x,prev_node->y),Eigen::Vector2f(node->x,node->y), 0x3ede12, global_viz_msg_);
    }
    first_point = false;
    prev_node = node;
  }
}

// Draw intersection points of line A to B that intersect circle C of radius r
// Adapted from code found at: 
// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
Eigen::Vector2f Navigation::GetIntersectionPoint(Eigen::Vector2f A, 
                                        Eigen::Vector2f B,
                                        Eigen::Vector2f C,
                                        float r) {
  
  Eigen::Rotation2Df rot(-robot_angle_);
  float Ax = A.x();
  float Ay = A.y();
  float Bx = B.x();
  float By = B.y();
  float Cx = C.x();
  float Cy = C.y();

  // compute the euclidean distance between A and B
  float LAB = sqrt(pow(Bx-Ax,2) + pow(By-Ay,2));

  // compute the direction vector D from A to B
  float Dx = (Bx-Ax) / LAB;
  float Dy = (By-Ay) / LAB;

  // compute the distance between the points A and E, where
  // E is the point of AB closest the circle center (Cx, Cy)
  float t = Dx * (Cx-Ax) + Dy * (Cy-Ay);    

  // compute the coordinates of the point E
  float Ex = t * Dx + Ax;
  float Ey = t * Dy + Ay;

  // compute the euclidean distance between E and C
  float LEC = sqrt(pow(Ex-Cx,2) + pow(Ey-Cy,2));

  // test if the line intersects the circle
  if (LEC < r) {
      float dt = sqrt(pow(r,2) - pow(LEC,2));

      // intersection point behind robot (not used)
      // float Fx = (t-dt) * Dx + Ax;
      // float Fy = (t-dt) * Dy + Ay;

      // intersection point in front of robot
      float Gx = (t+dt) * Dx + Ax;
      float Gy = (t+dt) * Dy + Ay;

      if ((((Gx >= Ax) && (Gx <= Bx)) || ((Gx <= Ax) && (Gx >= Bx))) &&
            (((Gy >= Ay) && (Gy <= By)) || ((Gy <= Ay) && (Gy >= By)))) {
        // path intersects circle in front of robot
        return rot * Eigen::Vector2f(Gx-robot_loc_.x(),Gy-robot_loc_.y());
      } else {
        // path only intersects circle behind robot
        return Eigen::Vector2f(0.0,0.0);
      }
  } else if( LEC == r ) { // else test if the line is tangent to circle
    // line is tangent to circle
    return Eigen::Vector2f(0.0,0.0);
  } else {
    // line doesn't intersect circle
    return Eigen::Vector2f(0.0,0.0);
  }    
}

Eigen::Vector2f Navigation::UpdateLocalTarget(){
  std::string curr_id;
  std::string prev_id;
  Eigen::Vector2f curr_node_loc;
  Eigen::Vector2f prev_node_loc;
  Eigen::Vector2f local_target;

  // handles case when end of global_path is within circle
  float dist_to_target = (robot_loc_ - Eigen::Vector2f(Nodes[global_path.back()]->x, Nodes[global_path.back()]->y)).norm();
  if (dist_to_target <= local_target_radius) {
    Eigen::Rotation2Df rot(-robot_angle_);
    local_target = rot * Eigen::Vector2f(global_target.x()-robot_loc_.x(),global_target.y()-robot_loc_.y());
    visualization::DrawCross(local_target, 0.15, 0x0045cf, local_viz_msg_);
    return local_target;
  }

  // handles case when global_path should intersect circle
  bool first_id = true;
  int i = 0;
  for (std::string node_id : global_path){
    curr_id = node_id;
    if (!first_id){
      prev_node_loc = Eigen::Vector2f(Nodes[prev_id]->x,Nodes[prev_id]->y);
      curr_node_loc = Eigen::Vector2f(Nodes[curr_id]->x,Nodes[curr_id]->y);
      local_target = GetIntersectionPoint(prev_node_loc, curr_node_loc, robot_loc_, local_target_radius); 

      if (local_target != Eigen::Vector2f(0.0,0.0)){
        // highlights line segment of intersection between path and circle
        // visualization::DrawLine(prev_node_loc, curr_node_loc, 0xcf0000, global_viz_msg_);
        visualization::DrawCross(local_target, 0.15, 0x0045cf, local_viz_msg_);
        return local_target;
      }
    }
    i++;
    first_id = false;
    prev_id = node_id;
  }
  
  // only returns when there is no intersection 
  // and we should replan global_path
  return Eigen::Vector2f(0.0, 0.0);
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
  InitializeOccupancyMatrix(map_);

  // Create ObjectAvoidance object that will manage path calculations
  path_planner_.reset(
    new object_avoidance::ObjectAvoidance(car_specs_, odd_num_paths, min_turn_radius_));
}

void Navigation::UpdateSimplePath(){
  global_path.clear();
  Eigen::Vector2i robot_id = PointToIndex(robot_loc_);
  global_path.push_back(to_string(robot_id.x()) + "," + to_string(robot_id.y()));
  Eigen::Vector2i target_id = PointToIndex(global_target);
  global_path.push_back(to_string(target_id.x()) + "," + to_string(target_id.y()));
}

void Navigation::UpdateGlobalPath(){
  global_path.clear();
  
  Eigen::Vector2i robot_id = PointToIndex(robot_loc_);
  string robot_name = to_string(robot_id.x()) +  "," + to_string(robot_id.y());
  Node* robot = Nodes[robot_name];

  Eigen::Vector2i target_id = PointToIndex(global_target);
  string target_name = to_string(target_id.x()) +  "," + to_string(target_id.y());
  Node* target = Nodes[target_name];
  
  string message = "Running A* with robot position " + robot_name + " and target " + target_name;
  ROS_INFO("%s", message.c_str());

  std::vector<std::string> result = a_star(robot, target, Nodes);
  /* std::vector<std::string> result = a_star_local_vars(robot, target, 
                                  Nodes, occupancy_matrix.rows(), occupancy_matrix.cols()); */
  
  message = "Ran A*, got result";
  for (string s : result) {
    message = message + " " + s;
  }

  ROS_INFO("%s \nMoving to Target.", message.c_str());
  global_path.insert(global_path.end(), result.begin(), result.end());
  global_path.push_back(target_name);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_complete_ = false;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  global_target = loc;

  UpdateGlobalPath(); // UpdateSimplePath();
}

void Navigation::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  ObstacleAvoid(); // Assignment 1 implementation

  // ==========VISUALIZATIONS/DEBUG=========
  // DrawOccupancyMatrix();
  // DrawMapPoints(map_);
  // DrawTargetNode(global_target);
  // DrawRobotNode(robot_loc_);
  visualization::DrawArc(Vector2f(0.0, 0.0), local_target_radius, 0, 2.0 * M_PI, 0x0045cf, local_viz_msg_);
  visualization::DrawLine(Eigen::Vector2f(0.0,0.0), local_target, 0x0045cf, local_viz_msg_);
  // visualization::DrawArc(robot_loc_, 2 * resolution, 0, 2.0 * M_PI, 0x3ede12, global_viz_msg_);
  if (global_target != Eigen::Vector2f(0.0,0.0)){
    DrawGlobalPath(); // ignores when global_target is (0.0,0.0)
                      // eg. when navigation is initialized.
  }
  
  // Debugging to see neighbors are accurate:
  // Eigen::Vector2i index = PointToIndex(global_target);
  // Node* n = Nodes[to_string(index.x()) + "," + to_string(index.y())];
  // DrawNeighbors(n);


  //========== MAIN NAVIGATION PLANNER LOOP=========
  // robot is at close global_target
  if ((global_target == Eigen::Vector2f(0.0,0.0)) || 
          ((robot_loc_ - global_target).norm() <= 2 * resolution)) { 
     local_target = Vector2f(0.0, 0.0);
     ROS_INFO("Arrived at Target!");
   } else { // robot is not at global target
      
      if (!global_path.empty()){
        local_target = UpdateLocalTarget();
      }

      // if the local_target returned (0.0, 0.0) it
      // means that we are too far from the global path
      // and it needs to be updated.
      if(local_target == Eigen::Vector2f(0.0,0.0)){
        UpdateGlobalPath();
        DrawGlobalPath();
        local_target = UpdateLocalTarget();
      }
   }
  
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
