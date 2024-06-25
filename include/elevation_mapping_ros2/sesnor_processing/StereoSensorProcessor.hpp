#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
class StereoSensorProcessor: public SensorProcessorBase
{
public:
    StereoSensorProcessor(const std::string& sensor_frame, const std::string& map_frame, const std::string& _robot_frame);
    ~StereoSensorProcessor();

    void readParameters(rclcpp::Node* _node) override;
private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;

    double depth_upper_limit_;
    double depth_lower_limit_;
    double coef_normal_variance_;
    double coef_lateral_variance_;
};
}