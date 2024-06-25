/**
 * @brief manage map data. 
 * raw_map_: not fused map. update this map.
 * fused_map_: raw_maps are fused to this map. 
*/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/iterators/EllipseIterator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <elevation_mapping_ros2/TypeDef.hpp>
#include <elevation_mapping_ros2/WeightedEmpiricalCumulativeDistributionFunction.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>

namespace elevation_mapping
{
template <typename Scalar>
struct VarianceClampOperator
{
    VarianceClampOperator(const Scalar& min, const Scalar& max) : min_variance_(min), max_variance_(max){}
    const Scalar operator()(const Scalar& x) const 
    {
        return x < min_variance_ ? min_variance_ : (x > max_variance_ ? std::numeric_limits<float>::infinity() : x);
    }
    Scalar min_variance_, max_variance_;
};

class ElevationMap
{
public:
    explicit ElevationMap();
    ~ElevationMap();

    bool add (PointCloudType::Ptr _point_cloud, Eigen::VectorXf& _variance, const rclcpp::Time& _time, const Eigen::Affine3d& _transform_sensor2map_);
    bool fuseAll();
    bool fuse(const grid_map::Index& _top_left_index, const grid_map::Size& size);
    bool update(const grid_map::Matrix& _variance, const grid_map::Matrix& _horizontal_variance_x, const grid_map::Matrix& _horizontal_variance_y, const grid_map::Matrix& _horizontal_variance_xy, const rclcpp::Time& _time_stamp);
    bool clear();
    void resetFusedMap();
    bool clean(); // cleans the elevation map data to stay within the specified bounds. 
    void move(const grid_map::Position& position);
    void visibilityCleanup(const rclcpp::Time& _time_stamp);
    bool extractVaildArea(const GridMap& _src_map, GridMap& _dst_map, const std::string& layer);

    void readParameters(rclcpp::Node* _node);
    
    void setGeometry(const grid_map::Length& _length, const double& _resolution, const grid_map::Position& position);
    void setFrameID(const std::string& frame_id);

    // getter
    const std::string& getFrameID() const;
    const rclcpp::Time getTimeOfLastUpdate(const rcl_clock_type_t type = RCL_ROS_TIME) const;
    GridMap& getRawMap();
    GridMap& getFusedMap();
private:
    static float cumulativeDistributionFunction(float x, float mean, float standardDeviation);


    std::shared_ptr<rclcpp::Clock> system_clock_;
    std::shared_ptr<rclcpp::Clock> ros_clock_;
    rclcpp::Time last_update_time_;
    rclcpp::Time initial_time_;
    rclcpp::Time scanning_duration_;

    GridMap raw_map_; //
    GridMap fused_map_; // update data by using covariance of map position
    GridMap visibility_clean_up_map_;

    // parameter
    float min_normal_variance_, max_normal_variance_;
    float min_horizontal_variance_, max_horizontal_variance_;
    float mahalanobis_distance_thres_;
    float increase_height_alpha_; // (0, 1)
    float multi_height_noise_;
    std::string logger_name_;
};

}