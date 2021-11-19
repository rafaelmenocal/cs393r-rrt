/*
 * Basic code for object detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include "object_avoidance.h"

#include <cmath>
#include <math.h>
#include "glog/logging.h"
#include "ros/ros.h"


#include "ros/ros.h"
#include "visualization/visualization.h"

namespace object_avoidance {

    /*
    * 
    * @param car_specs: the struct with information about the robot's dimensions
    * @param num_paths: the number of paths to consider for planning
    * @param min_turn_radius: the minimum radius this car can turn (TODO(alex): could be in CarSpecs?)
    */
    ObjectAvoidance::ObjectAvoidance(CarSpecs car_specs,
                                     int32_t num_paths,
                                     float_t min_turn_radius) : car_specs_(car_specs) {
        // Initialize all the empty paths.
        paths_ = std::make_shared<std::vector<PathOption>>(num_paths);
        // Loop through and create intial paths options.
        int middle_index = (num_paths - 1) / 2;
        float_t curvature_minimum = -1 / min_turn_radius;
        float_t curvature_increment = -curvature_minimum / middle_index;
        float_t curvature_current = curvature_minimum;
        for (int i = 0; i < num_paths; i++) {
            if (i == middle_index) {
                paths_->at(i).curvature = 0.0;
            } else {
                paths_->at(i).curvature = curvature_current;
            }
            curvature_current += curvature_increment;
        }
    };

    /*
    * Function to be called by clients whenever there is a new point cloud to use to update
    * the paths.
    * 
    * @param cloud: the point cloud reading
    */
    void ObjectAvoidance::UpdatePaths(const std::vector<Eigen::Vector2f>& cloud, const Eigen::Vector2f nav_target) {
        // Loop over all the candidate paths and update their stats based on the current
        // point cloud.
        for (auto& path : *paths_) {
            path.free_path_lengthv2 = FindMinPathLengthv2(cloud, path.curvature, nav_target);            
        }  
        CalculateClearances(5);
        // CalculateClearances(cloud);
        for (auto& path : *paths_) {
            path.score = score_max_distance_weight * path.free_path_lengthv2 + score_clearance_weight * path.clearance;
        }   
    }

    void ObjectAvoidance::CalculateClearances(const int num_neighbors) {
        float_t total;
        int count;
        int num_curvatures = paths_->size();
        for (int i = 0; i < num_curvatures; i++){
            total = 0;
            count = 0;
            for (int j = 0; j < (2 * num_neighbors + 1); j++){
                if ((i - num_neighbors + j >= 0) && (i - num_neighbors + j < num_curvatures)) {
                total += paths_->at(i - num_neighbors + j).free_path_lengthv2;
                count += 1;
                }
            }
            // paths_->at(i).clearance =  total / float_t(count);
            paths_->at(i).clearance = std::min(total / float_t(count), paths_->at(i).free_path_lengthv2);
        }
    }

    /*
    * Find the clearance for each path. This is the average path length of its 2 neighbors
    */
    void ObjectAvoidance::FindPathClearances() {
        auto idx_left = std::next(paths_->begin(), 1);
        auto idx_right = std::next(paths_->begin(), 2);
        auto second_to_last_idx = std::prev(paths_->end(), 2);
        for (auto curr_path = paths_->begin(); curr_path != paths_->end(); curr_path++) {
            curr_path->clearance += curr_path->free_path_length;
            curr_path->clearance += idx_left->free_path_length;
            curr_path->clearance += idx_right->free_path_length;
            curr_path->clearance /= 3.0;
            // If at the beginning of the paths, decrement the left index back to the first
            // path.
            if (curr_path == paths_->begin()) {
                idx_left--;
            // If at the second to last element, decrement the right element.
            } else if (curr_path == second_to_last_idx) {
                idx_right--;
            // Not at either end, so increment all iterators
            } else {
                idx_left++;
                idx_right++;
            }
        }
    }

    float_t ObjectAvoidance::FindMinPathLengthv2(
        const std::vector<Eigen::Vector2f>& cloud, float_t curvature, Eigen::Vector2f nav_target) {

        // maximum distance reading by laser
        float_t min_path_length = 10.0;
        float_t path_length;
        for (const auto& point: cloud) {
            // To avoid division by zero, send zero curvature path to special linear
            // calculator.
            if (curvature == 0) {
                path_length = FindStraightPathLength(point, nav_target); 
            } else {
                path_length = FindCurvePathLengthv2(point, curvature, nav_target);
            }
            // Update the longest path possible if a shorter one has been calculated.
            if (path_length < min_path_length) {
                min_path_length = path_length;
            }
        }
        return min_path_length;
    }

        /*
    * Find the longest traversable distance before collision along a straight path
    * 
    * @param point: the point to check for collision with
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    float_t ObjectAvoidance::FindStraightPathLength(const Eigen::Vector2f& point, Eigen::Vector2f nav_target) {
        float_t extra_front_safety = 0.0;
        float_t extra_side_safety = 0.1;
        float_t dist_to_target = GetDistance(nav_target, Eigen::Vector2f(0.0, 0.0));
        if ((abs(point.y()) <= car_specs_.total_side + extra_side_safety) && (point.x() > car_specs_.total_front + extra_front_safety)) {
            return std::min(dist_to_target, point.x() - car_specs_.total_front - extra_front_safety);
            // return point.x() - car_specs_.total_front - extra_front_safety;
        }  else { // point doesn't collide with front of car
            // return 10.0;
            return dist_to_target;
        }
    }

    /*
    * Find the highest scoring path
    * 
    * @return curvature of the highest scoring path
    */
    PathOption ObjectAvoidance::GetHighestScorePath() {
        return *std::max_element(
            paths_->begin(),
            paths_->end(),
            [](const PathOption& lhs, const PathOption& rhs){
                return lhs.score < rhs.score;});
    }

    bool PointWithinSafetyMargin(const Eigen::Vector2f proj_point,
                             float width, float length,
                             float axle_offset, float safety_margin_front, float safety_margin_side) {
        bool within_length = (proj_point.x() < (axle_offset + length + safety_margin_front)) && (proj_point.x() > (axle_offset - safety_margin_front));
        bool within_width = (proj_point.y() < (safety_margin_side + width/2.0)) && (proj_point.y() > (-safety_margin_side - width/2.0));
        return within_length && within_width;
    }

    float_t ObjectAvoidance::FindCurvePathLengthv2(const Eigen::Vector2f& point, float_t curvature, Eigen::Vector2f nav_target) {
        
        float_t extra_side_safety = 0.0;
        float_t extra_front_safety = 0.1;

        if (PointWithinSafetyMargin(point, car_specs_.car_width, car_specs_.car_length, car_specs_.rear_axle_offset, car_specs_.car_safety_margin_front, car_specs_.car_safety_margin_side)){
            return 10.0; //ignored as min path length point
        }

        float_t r = 1 / abs(curvature);
        Eigen::Vector2f turn_point = Eigen::Vector2f(0.0, r * int(curvature / abs(curvature)));
        float_t target_to_turnradius = GetDistance(turn_point, nav_target);
        float_t dist_to_target = GetDistance(nav_target, Eigen::Vector2f(0.0, 0.0));
        float_t max_theta = acos((pow(dist_to_target, 2) - pow(r, 2) - pow(target_to_turnradius, 2))/(-2.0 * r * target_to_turnradius));
        float_t max_arclength = abs(max_theta * r);

        float_t r_min = r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side - extra_side_safety;
        float_t r_mid = sqrt(pow(r_min, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front + extra_front_safety, 2)); 
        float_t r_max = sqrt(pow(r + (car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side + extra_side_safety, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front + extra_front_safety, 2)); 
        float_t r_obs = GetDistance(turn_point, point);
        
        float_t Beta;
        float_t alpha;
        if ((r_min <= r_obs) && (r_obs < r_mid)){ // point will hit the side of car
            Beta = acos((r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side - extra_side_safety) / r_obs);
        } else if ((r_mid <= r_obs) && (r_obs <= r_max)) {  // point will hit front of car
            Beta = asin((- car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front + extra_front_safety)/ r_obs);
        } else{ // else point doesn't hit car
            return max_arclength;
            // return std::min(float(10.0), float((r * M_PI/2))); //max_arclength;
        }
        float_t dist = GetDistance(point, Eigen::Vector2f(0.0, 0.0));
        alpha =  acos((pow(dist, 2) - pow(r_obs, 2) - pow(r, 2))/(-2 * r * r_obs)) - Beta;

        // return abs(alpha * r); 
        return std::min(abs(alpha * r), max_arclength);
    }

    // void ObjectAvoidance::CalculateClearances(const std::vector<Eigen::Vector2f>& cloud) {
    //     float_t extra_side_safety = 0.0;
    //     float_t extra_front_safety = 0.05;
    //     float_t r;
    //     Eigen::Vector2f turn_point;
    //     float_t alpha;
    //     float_t r_del_min;
    //     float_t r_del;
    //     float_t r_min;
    //     float_t r_max; 
    //     float_t r_obs;
    //     float_t dist_to_point;
    //     float_t point_alpha;

    //     for (auto& path : *paths_) {
    //         if (path.curvature == 0){
    //             r_del_min = 1000000;
    //             for (const auto& point: cloud) {
    //                 if ((abs(point.y()) > car_specs_.total_side + extra_side_safety) && ((point.x() < path.free_path_lengthv2) && (point.x() >= 0))){
    //                     r_del = abs(point.y()) - car_specs_.total_side + extra_side_safety;
    //                     if (r_del < r_del_min){
    //                         r_del_min = r_del;
    //                     }
    //                 }
    //             }
    //             path.clearance = r_del_min;
    //         } else {
    //             r = 1 / abs(path.curvature);
    //             turn_point = Eigen::Vector2f(0.0, r * int(path.curvature / abs(path.curvature)));
    //             alpha = path.free_path_lengthv2 * abs(path.curvature);
    //             r_del_min = r;
    //             r_min = r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side - extra_side_safety;
    //             r_max = sqrt(pow(r + (car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side + extra_side_safety, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front + extra_front_safety, 2));
                
    //             for (const auto& point: cloud) {
    //                 r_obs = GetDistance(turn_point, point);
    //                 dist_to_point = GetDistance(point, Eigen::Vector2f(0.0, 0.0));
    //                 point_alpha = acos((pow(dist_to_point, 2) - pow(r, 2) - pow(r_obs, 2))/(-2.0 * r * r_obs));
    //                 if (((r_obs <= r_min) || (r_obs >= r_max)) && (point_alpha < alpha) && (point.x() > 0)){
    //                     r_del = std::min(abs(r_min - r_obs), abs(r_obs - r_max));
    //                     if (r_del < r_del_min){
    //                         r_del_min = r_del;
    //                     }
    //                 }
    //             }
    //             if (r_del_min == r){
    //                 path.clearance = path.free_path_lengthv2;
    //             } else {
    //                 path.clearance = r_del_min;
    //             }
    //         }
    //     }
    // }

    /*
    * Function to find the longest traversable distance before collision along a given path
    * 
    * @param cloud: the point cloud reading
    * @param curvature: the path's curvature
    * 
    * @return the longest path along the given curve before any collision
    */
    // float_t ObjectAvoidance::FindMinPathLength(
    //     const std::vector<Eigen::Vector2f>& cloud, float_t curvature) {

    //     // maximum distance reading by laser
    //     float min_path_length = 10.0;
    //     float path_length;
    //     for (const auto& point: cloud) {
    //         // To avoid division by zero, send zero curvature path to special linear
    //         // calculator.
    //         if (curvature == 0) {
    //             path_length = FindStraightPathLength(point); 
    //         } else {
    //             path_length = FindCurvePathLength(point, curvature);
    //         }
    //         // Update the longest path possible if a shorter one has been calculated.
    //         if (path_length < min_path_length) {
    //             min_path_length = path_length;
    //         }
    //     }
    //     return min_path_length;
    // }

    /*
    * Find the longest traversable distance before collision along a curved path
    * 
    * @param point: the point to check for collision with
    * @param curvature: the curvature of the path
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    // float_t ObjectAvoidance::FindCurvePathLength(
    //     const Eigen::Vector2f& point, float_t curvature) {
        
    //     // If we are turning left and the point under consideration is below the bottom
    //     // of the car, we won't hit the point until we circle all the way back around.
    //     if (curvature > 0 && point[1] < -car_specs_.total_side) {
    //         return 10.0;
    //     }
    //     // If we are turning right and the point under consideration is above the top part
    //     // of the car, we won't hit the point until we circle all the way back around.
    //     else if (curvature < 0 && point[1] > car_specs_.total_side) {
    //         return 10.0;
    //     }

    //     float_t turning_radius = 1.0 / curvature;
    //     // Find the smallest radius of the car's swept volume. This point is along the inside
    //     // part of the car along the wheelbase.
    //     float_t inner_radius = abs(turning_radius) - car_specs_.total_side;
    //     // Find the radius drawn out by the inner most edge of the front part of the robot.
    //     // This determines if a point will collide with the front or the inner side of the
    //     // car.
    //     float_t middle_radius = sqrt(
    //         pow(abs(turning_radius) - car_specs_.total_side, 2.0) + 
    //         pow(car_specs_.total_front, 2.0));
    //     // Find the radius drawn out by the outter most edge of the front part of the robot.
    //     // This determines if a point will collide with the front of the car.
    //     float_t outter_radius = sqrt(
    //         pow(abs(turning_radius) + car_specs_.total_side, 2.0) +
    //         pow(car_specs_.total_front, 2.0));

    //     float_t shortest_distance = 10.0;
    //     Eigen::Vector2f furthest_point;

    //     // Find the distance from the center of turning circle to point
    //     float_t dist_to_point = GetDistance(0, turning_radius, point[0], point[1]);

    //     float_t collision_to_point;
    //     // collision along inside part of the car
    //     if (inner_radius <= dist_to_point && dist_to_point < middle_radius) {
    //         // Find the x-coordinate of the point of collision along the car. We know the
    //         // y-position of this point. This works for positive and negative curvature.
    //         float_t x = sqrt(
    //             pow(dist_to_point, 2) - pow(abs(turning_radius) - car_specs_.total_side, 2));
        
    //         // Find the L2 distance from the point of collision to where the obstacle
    //         // point is right now. This will be used for arclength calculations.
    //         if (turning_radius < 0) {
    //             // If we are turning right, the collision along the inside of the car
    //             // will have a negative y component.
    //             collision_to_point = GetDistance(
    //                 x, -car_specs_.total_side, point[0], point[1]);
    //         }
    //         else {
    //             // If turning left, the collision along the inside wall will have a
    //             // positive y component.
    //             collision_to_point = GetDistance(
    //                 x, car_specs_.total_side, point[0], point[1]);
    //         }
    //         // Arch length is equal to r*theta where
    //         // theta := arccos(1 - (collision_to_point^2) / (2 * dist_to_point^2))
    //         float_t arc_length = dist_to_point * 
    //             acos(1 - (pow(collision_to_point, 2.0) / (2 * pow(dist_to_point, 2.0))));
    //         // Keep track of whether this point gives a smaller possible distance to travel.
    //         if (arc_length <= shortest_distance) {
    //             shortest_distance = arc_length;
    //             furthest_point = point;
    //         }
    //     // collision along front of the car
    //     } else if (middle_radius <= dist_to_point && dist_to_point < outter_radius) {
    //         float_t y;

    //         if (turning_radius < 0) {
    //             y = sqrt(
    //                 pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0)) - abs(turning_radius);
    //         } else {
    //             y = turning_radius - sqrt(pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0));
    //         }
    //         // Calculate the distance from the point of collision along the car to the
    //         // current position of the car.
    //         float_t dist_from_collision_to_point = GetDistance(
    //             car_specs_.total_front, y, point[0], point[1]);
    //         float_t angle = acos(
    //             1 - pow(dist_from_collision_to_point, 2.0) / (2 * (pow(dist_to_point, 2.0))));
    //         float_t arc_length = dist_to_point * angle;

    //         if (arc_length <= shortest_distance) {
    //             shortest_distance = arc_length;
    //             furthest_point = point;
    //         }
    //     }
    //     return shortest_distance;
    // }

};
