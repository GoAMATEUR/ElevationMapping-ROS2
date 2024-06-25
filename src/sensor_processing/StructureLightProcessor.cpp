#include <elevation_mapping_ros2/sesnor_processing/StructureLightProcessor.hpp>
// #include <elevation_mapping_ros2/sensor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
StructureLightProcessor::StructureLightProcessor(const std::string& _sensor_frame, const std::string& _map_frame, const std::string& _robot_frame)
    : SensorProcessorBase(_sensor_frame, _map_frame, _robot_frame)
{

}

StructureLightProcessor::~StructureLightProcessor() {}

bool StructureLightProcessor::filterSensorType(PointCloudType::Ptr _point_cloud)
{
    pcl::PassThrough<PointType> pass_through_filter(true);
    pass_through_filter.setInputCloud(_point_cloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(cutoff_min_depth_, 
                                        cutoff_max_depth_);

    PointCloudType temp_point_cloud;
    pass_through_filter.filter(temp_point_cloud);
    _point_cloud->swap(temp_point_cloud);

    return true;
}

void StructureLightProcessor::computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
{
    _variance.resize(_point_cloud->size());

    // projection vector
    const Eigen::RowVector3f projection_vector = Eigen::RowVector3f::UnitZ();

    // sensor to map jacobian
    const Eigen::RowVector3f sensor_jac = projection_vector*(rotation_map2base_.transpose()*rotation_base2sensor_.transpose()).cast<float>();

    // Robot rotation covariance
    const Eigen::Matrix3f rotation_variance = _robot_covariance.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation jacobian
    const Eigen::Matrix3f R_B2M_transpose = rotation_map2base_.transpose().cast<float>();
    const Eigen::RowVector3f Pro_R_B2M_transpose = projection_vector*R_B2M_transpose;
    const Eigen::Matrix3f R_S2B_transpose = rotation_base2sensor_.transpose().cast<float>();
    const Eigen::Matrix3f T_B2S_skew = computeSkewMatrixfromVector(translation_base2sensor_.cast<float>());

    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        const auto point{_point_cloud->points[i]};
        Eigen::Vector3f point_vector{point.x, point.y, point.z};
        float height_variance = 0.0;

        // float measurement_distance2 = std::pow(point.x, 2.0)+std::pow(point.y, 2.0) + std::pow(point.z, 2.0);
        float measurement_distance = point.z;
        // compute sensor covariance matirx
        // const float deviationNormal =
        //     parameters.sensorParameters_.at("normal_factor_a") +
        //     parameters.sensorParameters_.at("normal_factor_b") * (measurementDistance - parameters.sensorParameters_.at("normal_factor_c")) *
        //         (measurementDistance - parameters.sensorParameters_.at("normal_factor_c")) +
        //     parameters.sensorParameters_.at("normal_factor_d") * pow(measurementDistance, parameters.sensorParameters_.at("normal_factor_e"));
        const float  deviationNormal = normal_factor_a_ + 
            normal_factor_b_ * (measurement_distance - normal_factor_c_) * (measurement_distance - normal_factor_c_) + normal_factor_d_ * std::pow(measurement_distance, normal_factor_e_);
        const float varianceNormal = deviationNormal * deviationNormal;
        const float deviationLateral = lateral_factor_ * measurement_distance;
        const float varianceLateral = deviationLateral * deviationLateral;
        Eigen::Matrix3f sensor_variance = Eigen::Matrix3f::Zero();
        sensor_variance.diagonal() << varianceLateral, varianceLateral, varianceNormal;
        
        // robot rotation jacobian
        const Eigen::Matrix3f T_S2P_inB_skew = computeSkewMatrixfromVector(R_S2B_transpose*point_vector); // R_S2B_transpose_times_T_B2S_skew
        const Eigen::RowVector3f rotation_jac = Pro_R_B2M_transpose*(T_S2P_inB_skew+T_B2S_skew); // jacobian  d{(projection)*(B2P in map frame)}/d{rotation angle} 

        // variance for map 
        height_variance = rotation_jac*rotation_variance*rotation_jac.transpose();
        height_variance += sensor_jac*sensor_variance*sensor_jac.transpose();

        _variance(i) = height_variance;
        assert( _variance(i) >= 0.0 && "Variance of point cloud is lower than 0");
    }
}

void StructureLightProcessor::readParameters(rclcpp::Node* _node)
{
    param_voxel_grid_fitler_.use_filter = _node->declare_parameter("sensor.use_voxel_filter", true);
    param_voxel_grid_fitler_.leaf_size = _node->declare_parameter("sensor.voxel_leaf_size", 5.0);
    param_pass_through_filter_.lower_threshold_ = _node->declare_parameter("sensor.pass_filter_lower_threshold", -std::numeric_limits<double>::infinity());
    param_pass_through_filter_.upper_threshold_ = _node->declare_parameter("sensor.pass_filter_upper_threshold", std::numeric_limits<double>::infinity());
    logger_name_ = _node->declare_parameter("sensor.logger_name", "StructureLightProcessor");

    normal_factor_a_ = _node->declare_parameter("sensor_processor/normal_factor_a", 0.0);
    normal_factor_b_ = _node->declare_parameter("sensor_processor/normal_factor_b", 0.0);
    normal_factor_c_ = _node->declare_parameter("sensor_processor/normal_factor_c", 0.0);
    normal_factor_d_ = _node->declare_parameter("sensor_processor/normal_factor_d", 0.0);
    normal_factor_e_ = _node->declare_parameter("sensor_processor/normal_factor_e", 0.0);
    lateral_factor_ = _node->declare_parameter("sensor_processor/lateral_factor", 0.0);
    cutoff_min_depth_ = _node->declare_parameter("sensor_processor/cutoff_min_depth", std::numeric_limits<double>::min());
    cutoff_max_depth_ = _node->declare_parameter("sensor_processor/cutoff_max_depth", std::numeric_limits<double>::max());
}

}