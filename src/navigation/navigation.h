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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "object_avoidance.h"
// #include "global_planner.h"
// #include "graph.h"
#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

struct Node {
    std::string id; // unique node "x,y,theta"
    Eigen::Vector2f loc; // location of node
    float_t theta; // final heading at location
    std::string parent_id; // "x,y,theta" of parent node
    Eigen::Vector2f center_of_turn; // center of turn
    float_t radius; // radius of turning arc
    float_t theta_start; // start theta from COT to parent_node
    float_t theta_end; // end theta from COT to this node
    Eigen::Vector2f sampled_point; 
  };

namespace navigation {

class Navigation {
 public:

  // --------- Added ------------
  Eigen::Vector2f last_odom_loc_ = Eigen::Vector2f(0.0, 0.0);
  float last_odom_angle_ = 0.0;
  Eigen::Vector2f odom_vel_  = Eigen::Vector2f(0.0, 0.0);
  Eigen::Vector2f last_odom_vel_ = Eigen::Vector2f(0.0, 0.0);
  Eigen::Vector2f odom_accel_ = Eigen::Vector2f(0.0, 0.0);
  // -- Kinematic and dynamic constraints for the car.
  float max_vel_ = 0.0;
  float max_accel_ = 4.0;
  float min_turn_radius_ = 0.98;
  // -- Car dimensions.
  float car_width_ = 0.281;
  float car_length_ = 0.535;
  float car_height_ = 0.15;
  float car_safety_margin_front_ = 0.2;
  float car_safety_margin_side_ = 0.05;
  // -- Location of the robot's rear wheel axle relative to the center of the body.
  float rear_axle_offset_ = -0.162;
  object_avoidance::CarSpecs car_specs_ = {car_width_, car_height_,
                         car_length_, car_safety_margin_front_,
                         car_safety_margin_side_, rear_axle_offset_};
  
  Eigen::Vector2f laser_loc_ = Eigen::Vector2f(0.2, 0.15);
  // -- Simulation Update Frequency.
  float update_frequency_ = 20.0;

  /*
  * Determine if a point is within the circles defined by the minimum turning radius.
  * The center point of each of these circles is at (+- min_turn_radius_, 0) where
  * y is the baselink of the car.
  */
  inline bool FeasiblePoint(const Eigen::Vector2f& sampled_point) {
    // Just calculate the point as if all are to the "left of the car", meaning positive
    // curvature.
    Eigen::Vector2f sampled_point_abs(abs(sampled_point.x()), sampled_point.y());
    Eigen::Vector2f center(min_turn_radius_, 0.0);

    if ((sampled_point_abs - center).squaredNorm() < min_turn_radius_) {
      // Sampled point within the circular region the car cannot reach.
      return false;
    }
    return true;
  }  

    // Constructor
  explicit Navigation(const std::string& map_file, const double& latency, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();

  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void ObstacleAvoid();

  void DrawCar();

  void DrawTarget(bool& found);

  void FindPathToGoal();

  void GenerateGraphSolution();

  Node ProcessSampledPoint(Eigen::Vector2f& sample_point);

  vector_map::VectorMap map_;

  bool MapStraightLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point);

  bool MapCurvedLineIntersection(Eigen::Vector2f& loc, Eigen::Vector2f& point);

  Node ProcessSampledPointFromParent(Eigen::Vector2f& sample_point, std::string parent_id);

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  float odom_omega_;

  double latency;

  std::unique_ptr<object_avoidance::ObjectAvoidance> path_planner_;
  
};

}  // namespace navigation

#endif  // NAVIGATION_H
