#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

SensorProcessorBase::SensorProcessorBase(
    const std::string& _sensor_frame, const std::string& _map_frame, const std::string& _robot_frame)
    : kSensorFrameID_(_sensor_frame), kMapFrameID_(_map_frame), kRobotFrameID_(_robot_frame)
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);   
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::process(const sensor_msgs::msg::PointCloud2::UniquePtr& _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance,  PointCloudType::Ptr& _processed_point_cloud_map_frame, Eigen::VectorXf& _variance)
{
    // Convert to pcl point cloud
    PointCloudType point_cloud;
    pcl::fromROSMsg(*_point_cloud, point_cloud);
    current_time_point_ = tf2_ros::fromMsg(_point_cloud->header.stamp);
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Input point cloud size: %d", point_cloud.points.size());

    // listening transform from sensor frame to map frame
    if (!updateTransformations()) return false;

    // transform into sensor frame
    PointCloudType::Ptr point_cloud_sensor_frame(new PointCloudType);
    if (!transformPointCloud(point_cloud, point_cloud_sensor_frame, kSensorFrameID_)) return false;

    // remove Nans 
    if (!removeNans(point_cloud_sensor_frame)) return false;

    // reduce points using voxel grid filter
    if (!reducePoint(point_cloud_sensor_frame)) return false;

    // specific filtering per sensor type
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Filtering according to sensor type");
    if (!filterSensorType(point_cloud_sensor_frame)) return false;

    // transform into map frame
    if (!transformPointCloud(*point_cloud_sensor_frame, _processed_point_cloud_map_frame, kMapFrameID_)) return false;

    // remove outside limits in map frame
    std::vector<PointCloudType::Ptr> point_cloud_list{point_cloud_sensor_frame, _processed_point_cloud_map_frame};
    if (!removeOutsideLimits(_processed_point_cloud_map_frame, point_cloud_list)) return false;

    // compute variance 
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Compute variance");
    computeVariance(point_cloud_sensor_frame, _robot_covariance, _variance);

    // RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Output point cloud size: %d", _processed_point_cloud_map_frame->points.size());
    return true;
}

bool SensorProcessorBase::updateTransformations()
{
    try {  
        // sensor to map
        geometry_msgs::msg::TransformStamped transformTF = tf_buffer_->lookupTransform(kSensorFrameID_, kMapFrameID_, current_time_point_, tf2::durationFromSec(1.0));
        transform_sensor2map_= tf2::transformToEigen(transformTF);
        
        // base to sensor
        transformTF = tf_buffer_->lookupTransform(kRobotFrameID_, kSensorFrameID_, current_time_point_, tf2::durationFromSec(1.0));
        Eigen::Affine3d transform_affine;
        transform_affine = tf2::transformToEigen(transformTF);
        rotation_base2sensor_ = transform_affine.rotation().matrix();
        translation_base2sensor_ = transform_affine.translation();

        // map to base
        transformTF = tf_buffer_->lookupTransform(kMapFrameID_, kRobotFrameID_, current_time_point_, tf2::durationFromSec(1.0));
        Eigen::Affine3d transform_tf_affine;
        transform_tf_affine = tf2::transformToEigen(transformTF);
        rotation_map2base_ = transform_tf_affine.rotation().matrix();
        translation_map2base_ = transform_tf_affine.translation();

        if (!first_tf_available_) first_tf_available_ = true;
        return true;
    }
    catch(const tf2::TransformException& e)
    {
        if (!first_tf_available_) {
            return false;
        }
        RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "%s", e.what());
        return false;
    }
    
}

bool SensorProcessorBase::transformPointCloud(const PointCloudType& _point_cloud, PointCloudType::Ptr& _transformed_point_cloud, const std::string& _target_frame)
{
    const std::string& input_frame = _point_cloud.header.frame_id;
    geometry_msgs::msg::TransformStamped transformTF;
    try {
        transformTF = tf_buffer_->lookupTransform(_target_frame, input_frame, current_time_point_, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "%s", ex.what());
        return false;
    }
    Eigen::Affine3f transform = tf2::transformToEigen(transformTF).cast<float>();
    pcl::transformPointCloud(_point_cloud, *_transformed_point_cloud, transform);
    _transformed_point_cloud->header.frame_id = _target_frame;
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Point cloud is transformed from %s to %s", _point_cloud.header.frame_id.c_str(),  _transformed_point_cloud->header.frame_id.c_str());
    return true;
}

bool SensorProcessorBase::removeNans(PointCloudType::Ptr _point_cloud)
{
    pcl::Indices indices;
    PointCloudType temp_point_cloud;
    if (!_point_cloud->is_dense)
    {
        pcl::removeNaNFromPointCloud(*_point_cloud, temp_point_cloud, indices);
        _point_cloud->swap(temp_point_cloud);
        return true;
    }
    return true;
}

bool SensorProcessorBase::reducePoint(PointCloudType::Ptr _point_cloud)
{
    if (param_voxel_grid_fitler_.use_filter)
    {
        PointCloudType temp_point_cloud;
        pcl::VoxelGrid<PointType> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(_point_cloud);
        voxel_grid_filter.setLeafSize(param_voxel_grid_fitler_.leaf_size, param_voxel_grid_fitler_.leaf_size, param_voxel_grid_fitler_.leaf_size);
        voxel_grid_filter.filter(temp_point_cloud);
        _point_cloud->swap(temp_point_cloud);
        return true;
    }
    else return true;
}

bool SensorProcessorBase::removeOutsideLimits(const PointCloudType::Ptr& _reference, std::vector<PointCloudType::Ptr>& _point_clouds)
{
    if (!std::isfinite(param_pass_through_filter_.lower_threshold_) && !std::isfinite(param_pass_through_filter_.upper_threshold_))
    {
        RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "pass through filter is not applied");
        return true;
    }
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Limiting point cloud to the height interval of [%lf, %lf] relative to the robot base", param_pass_through_filter_.lower_threshold_, param_pass_through_filter_.upper_threshold_);

    pcl::PassThrough<PointType> pass_through_filter(true);
    pass_through_filter.setInputCloud(_reference);
    pass_through_filter.setFilterFieldName("z");
    double lower = translation_map2base_.z() + param_pass_through_filter_.lower_threshold_;
    double upper = translation_map2base_.z() + param_pass_through_filter_.upper_threshold_;
    pass_through_filter.setFilterLimits(lower, upper);
    pcl::IndicesPtr inside_indeces(new std::vector<int>);
    pass_through_filter.filter(*inside_indeces);

    for (auto& point_cloud: _point_clouds)
    {
        pcl::ExtractIndices<PointType> extract_indices_filter;
        extract_indices_filter.setInputCloud(point_cloud);
        extract_indices_filter.setIndices(inside_indeces);
        PointCloudType temp_point_cloud;
        extract_indices_filter.filter(temp_point_cloud);
        point_cloud->swap(temp_point_cloud);
    }
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Remove point out side limits. Reduced point cloud to %ld points", (_point_clouds[0]->size()));
    return true;
}

Eigen::Matrix3f SensorProcessorBase::computeSkewMatrixfromVector(const Eigen::Vector3f& _vec)
{
    Eigen::Matrix3f mat;
    mat.setZero();
    mat(0, 1) =-_vec.z();
    mat(0, 2) = _vec.y();
    mat(1, 0) = _vec.z();
    mat(1, 2) =-_vec.x();
    mat(2, 0) =-_vec.y();
    mat(2, 1) = _vec.z();

    return mat;
}


}

