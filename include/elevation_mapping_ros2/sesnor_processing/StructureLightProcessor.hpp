#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
class StructureLightProcessor: public SensorProcessorBase
{
public:
    StructureLightProcessor(const std::string& sensor_frame, const std::string& map_frame, const std::string& _robot_frame);
    ~StructureLightProcessor();

    void readParameters(rclcpp::Node* _node) override;
private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;

    // Sensor specific parameters
    double normal_factor_a_;
    double normal_factor_b_;
    double normal_factor_c_;
    double normal_factor_d_;
    double normal_factor_e_;
    double lateral_factor_;
    double cutoff_min_depth_;
    double cutoff_max_depth_;
};
}