#pragma once

#include <rclcpp/rclcpp.hpp>

#include <elevation_mapping_ros2/TypeDef.hpp>
#include <elevation_mapping_ros2/ElevationMap.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/EulerAngles>

namespace elevation_mapping
{

class RobotMotionUpdater
{
public:
    using PoseTransform = Eigen::Affine3d;
    using PoseCovariance = Eigen::Matrix<double, 6, 6>;
    using ReducedCovariance = Eigen::Matrix<double, 4, 4>;
    RobotMotionUpdater();
    ~RobotMotionUpdater();
    
    bool update(ElevationMap& _map, const PoseTransform& _pose_transform, const PoseCovariance& _pose_covariance, const rclcpp::Time& _time_stamp);
    void readParameters(rclcpp::Node* _node);
private:

    bool computeReducedCovariance(const PoseTransform& _pose_transform, const PoseCovariance& _pose_covariance, ReducedCovariance& _reduced_covariance);
    bool computeRelativeCovariance(const PoseTransform& _pose_transform, const ReducedCovariance& _reduced_covariance, ReducedCovariance& _relative_covariance);
    Eigen::Matrix3d computeSkewMatrixfromVector(const Eigen::Vector3d& _vec);
    
    // time 
    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Time pre_update_time_;

    // transform
    Eigen::Matrix<double, 4, 4> pre_reduced_covariance_;
    Eigen::Affine3d pre_robot_pose_;

    // params
    std::string logger_name_;

};
}