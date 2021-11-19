#ifndef __SRC_NAVIGATION_OBJECT_AVOIDANCE__
#define __SRC_NAVIGATION_OBJECT_AVOIDANCE__

#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"


namespace object_avoidance {

    struct CarSpecs {
        float_t car_width;
        float_t car_height;
        float_t car_length;
        float_t car_safety_margin_front;
        float_t car_safety_margin_side;
        // This is the rear axle's offset from the center of the car
        float_t rear_axle_offset;

        // Calculated params given the robot's dimensions
        float_t total_front;
        float_t total_back;
        float_t total_side;

        CarSpecs(float_t car_width, float_t car_height,
                 float_t car_length, float_t car_safety_margin_front,
                 float_t car_safety_margin_side, float_t rear_axle_offset) :
                 car_width(car_width),
                 car_height(car_height),
                 car_length(car_length),
                 car_safety_margin_front(car_safety_margin_front),
                 car_safety_margin_side(car_safety_margin_side),
                 rear_axle_offset(rear_axle_offset)
            {
                total_front = -rear_axle_offset + (car_length / 2.0) + car_safety_margin_front;
                total_back = -rear_axle_offset - (car_length / 2.0) - car_safety_margin_front;
                total_side = (car_width / 2.0) + car_safety_margin_side;
            }

    };

    struct PathOption {
        float_t curvature;
        float_t clearance;
        float_t free_path_length;
        float_t free_path_lengthv2;
        float_t score;
        float_t turn_magnitude;
        Eigen::Vector2f obstruction;
        Eigen::Vector2f closest_point;
    };

    // Just for making typing easier.
    typedef std::shared_ptr<std::vector<PathOption>> paths_ptr;

    class ObjectAvoidance {

        private:
            // Used to keep a copy of all relevant robot car dimensions.
            CarSpecs car_specs_;
            // Keep a master list of all the candidate path options.
            paths_ptr paths_;
            // weight the max distance twice as much as not wanting to turn
            float_t score_max_distance_weight = 1.0;
            // float_t score_min_turn_weight = 0.5;
            float_t score_clearance_weight = 4.0;
        
            float_t FindMinPathLengthv2(const std::vector<Eigen::Vector2f>& cloud, float_t curvature, Eigen::Vector2f nav_target);
            float_t FindStraightPathLength(const Eigen::Vector2f& point, Eigen::Vector2f nav_target);
            inline float_t GetDistance(float_t x0, float_t y0, float_t x1, float_t y1) {
                return sqrt(pow(x1 - x0, 2.0) + pow(y1 - y0, 2.0));
            }
            inline float_t GetDistance(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2){
                return sqrt(pow(point2.x() - point1.x(), 2) + pow(point2.y() - point1.y(), 2));
            }
            float_t FindCurvePathLengthv2(const Eigen::Vector2f& point, float_t curvature, Eigen::Vector2f nav_target);
            void FindPathClearances();
            void CalculateClearances(const int num_neighbors);
            
            // float_t FindCurvePathLength(const Eigen::Vector2f& point, float_t curvature);
            // float_t FindMinPathLength(const std::vector<Eigen::Vector2f>& cloud, float_t curvature);
            // void CalculateClearances(const std::vector<Eigen::Vector2f>& cloud);


        public:
            // Constructor. Initialize the difference paths here.
            explicit ObjectAvoidance(CarSpecs car_specs,
                                     int32_t num_paths,
                                     float_t min_turn_radius);
            // To be called whenever there is a new point cloud reading.
            void UpdatePaths(const std::vector<Eigen::Vector2f>& cloud, Eigen::Vector2f nav_target);
            // Simple public getter function for accessing candidate paths.
            inline paths_ptr GetPaths() {return paths_;};
            // Return the curvature with the highest score.
            PathOption GetHighestScorePath();
    };

} // namespace object_avoidance

#endif // __SRC_NAVIGATION_OBJECT_AVOIDANCE__