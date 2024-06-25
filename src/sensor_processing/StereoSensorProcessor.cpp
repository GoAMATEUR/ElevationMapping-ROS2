#include <elevation_mapping_ros2/sesnor_processing/StereoSensorProcessor.hpp>
// #include <elevation_mapping_ros2/sensor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
StereoSensorProcessor::StereoSensorProcessor(const std::string& _sensor_frame, const std::string& _map_frame, const std::string& _robot_frame)
    : SensorProcessorBase(_sensor_frame, _map_frame, _robot_frame)
{

}

StereoSensorProcessor::~StereoSensorProcessor() {}

bool StereoSensorProcessor::filterSensorType(PointCloudType::Ptr _point_cloud)
{
    pcl::PassThrough<PointType> pass_through_filter(true);
    pass_through_filter.setInputCloud(_point_cloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(depth_lower_limit_, depth_upper_limit_);

    PointCloudType temp_point_cloud;
    pass_through_filter.filter(temp_point_cloud);
    _point_cloud->swap(temp_point_cloud);

    return true;
}

void StereoSensorProcessor::computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
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

        float measurement_distance2 = std::pow(point.x, 2.0)+std::pow(point.y, 2.0) + std::pow(point.z, 2.0);

        // compute sensor covariance matirx
        float variance_normal = coef_normal_variance_* measurement_distance2;
        float variance_lateral = coef_lateral_variance_*measurement_distance2;
        Eigen::Matrix3f sensor_variance = Eigen::Matrix3f::Zero();
        sensor_variance.diagonal() << variance_lateral, variance_lateral, variance_normal;
        
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

void StereoSensorProcessor::readParameters(rclcpp::Node* _node)
{
    param_voxel_grid_fitler_.use_filter = _node->declare_parameter("sensor.use_voxel_filter", true);
    param_voxel_grid_fitler_.leaf_size = _node->declare_parameter("sensor.voxel_leaf_size", 5.0);
    param_pass_through_filter_.lower_threshold_ = _node->declare_parameter("sensor.pass_filter_lower_threshold", -std::numeric_limits<double>::infinity());
    param_pass_through_filter_.upper_threshold_ = _node->declare_parameter("sensor.pass_filter_upper_threshold", std::numeric_limits<double>::infinity());
    logger_name_ = _node->declare_parameter("sensor.logger_name", "StereoSensorProcessor");

    depth_upper_limit_ = _node->declare_parameter("depth_upper_limit", 20.0);
    depth_lower_limit_ = _node->declare_parameter("depth_lower_limit", 0.1);
    coef_normal_variance_ = _node->declare_parameter("coef_normal_variance", 0.0001); 
    coef_lateral_variance_ = _node->declare_parameter("coef_lateral_variance", 0.0001);
}

}