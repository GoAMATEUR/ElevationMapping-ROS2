#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

class PerfectSensorProcessor : public SensorProcessorBase
{
public: 
    explicit PerfectSensorProcessor(const std::string _sensor_frame, const std::string _map_frame, const std::string& _robot_frame);

    ~PerfectSensorProcessor();
    void readParameters(rclcpp::Node* _node) override;
private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;
};  

}