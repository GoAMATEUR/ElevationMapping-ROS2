#include <elevation_mapping_ros2/ElevationMapping.hpp>

namespace elevation_mapping {

using std::placeholders::_1;

ElevationMapping::ElevationMapping(const rclcpp::NodeOptions options) 
    // : rclcpp::Node("elevation_mapping", rclcpp::NodeOptions().use_intra_process_comms(true))
    : rclcpp::Node("elevation_mapping", options)
{
    readParameters();
    // create tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // subscriber
    sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "elevation_mapping/input/point_cloud", 10, std::bind(&ElevationMapping::callbackPointcloud, this, _1)
    );

    pub_raw_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "elevation_mapping/output/raw_map", 10
    );  
    pub_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "elevation_mapping/output/processed_point_cloud", 100
    );

    if (use_pose_update_)
    {
        RCLCPP_INFO(get_logger(), "Updation by pose message is enabled");
        // std::string input_pose_topic = declare_parameter("input.pose_covariance");
        sub_pose_.subscribe(this, "elevation_mapping/input/pose");
        pose_cache_.connectInput(sub_pose_);
        pose_cache_.setCacheSize(pose_cache_size_);
    }
    else 
    {
        RCLCPP_INFO(get_logger(), "Updation by pose message is disable");
    }
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

ElevationMapping::~ElevationMapping() {}

void ElevationMapping::callbackPointcloud(const sensor_msgs::msg::PointCloud2::UniquePtr _point_cloud)
{
    std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
    last_point_cloud_update_time_ = rclcpp::Time(_point_cloud->header.stamp, RCL_ROS_TIME);

    Eigen::Matrix<double, 6, 6> robot_pose_covariance;
    robot_pose_covariance.setZero();
    if (use_pose_update_)
    {
        std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(last_point_cloud_update_time_);
        if (!pose_msg) 
        {
            if (pose_cache_.getOldestTime() > last_point_cloud_update_time_)
            {
                RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", pose_cache_.getOldestTime().seconds(), last_point_cloud_update_time_.seconds());
            }
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f", last_point_cloud_update_time_.seconds());
            }
            return;
        }
        robot_pose_covariance = Eigen::Map<const Eigen::MatrixXd>(pose_msg->pose.covariance.data(), 6, 6);
    }

    // process point cloud
    PointCloudType::Ptr point_cloud_map_frame(new PointCloudType);
    Eigen::VectorXf height_variance;
    if (!sensor_processor_->process(_point_cloud, robot_pose_covariance, point_cloud_map_frame, height_variance))
    {
        if (!sensor_processor_->isFirstTfAvailable())
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Waiting for tf transformation to be available. (Message is throttled. 10s)");
            return;
        }
        else 
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Point cloud could not be processed. (Throttled 10s)");
            return;
        }
    }
    // pcl::PCLPointCloud2 pcl_pc;
    // sensor_msgs::msg::PointCloud2 processed_point_cloud;
    // pcl::toPCLPointCloud2(*point_cloud_map_frame, pcl_pc);
    // pcl_conversions::fromPCL(pcl_pc, processed_point_cloud);
    // pub_point_cloud_->publish(processed_point_cloud);

    updateMapLocation();

    if (!updatePrediction(last_point_cloud_update_time_))
    {
        RCLCPP_ERROR(get_logger(), "Updating process noise failed");
        return ;
    }

    // add point cloud to elevation map
    if (!map_.add(point_cloud_map_frame, height_variance, last_point_cloud_update_time_, sensor_processor_->getTransformSensor2Map()))
    {
        RCLCPP_ERROR(get_logger(), "Adding point cloud to elevation map failed");
        // reset map update timer
        return ;
    }    

    // map_.getRawMap().get("elevation").setZero();
    // map_.getRawMap().get("variance").setZero();

    // fuse previous map and current map
    // if (use_visibility_clean_up_) map_.visibilityCleanup(last_point_cloud_update_time_);
    // map_.fuseAll();
    if (extract_vaild_area_)
    {
        GridMap map_pub;
        if (!map_.extractVaildArea(map_.getRawMap(), map_pub, "elevation")) {
            // RCLCPP_INFO(get_logger(), "Failed to get submap information");
            return;
        }
        grid_map_msgs::msg::GridMap::UniquePtr message(new grid_map_msgs::msg::GridMap);
        message = grid_map::GridMapRosConverter::toMessage(map_pub, std::vector<std::string>{"elevation", "variance"});
        // RCLCPP_INFO(get_logger(), "publish map address: 0x%x", &(message->data));
        pub_raw_map_->publish(std::move(message));
    }
    else
    {
        grid_map_msgs::msg::GridMap::UniquePtr message(new grid_map_msgs::msg::GridMap);
        message = grid_map::GridMapRosConverter::toMessage(map_.getRawMap(), std::vector<std::string>{"elevation", "variance"});
        // RCLCPP_INFO(get_logger(), "publish map address: 0x%x", &(message->data));
        pub_raw_map_->publish(std::move(message));
    }
    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    RCLCPP_INFO(get_logger(), "elevation mapping processing time: %lf", elapsed);
}

bool ElevationMapping::updateMapLocation()
{
    RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization");

    geometry_msgs::msg::TransformStamped track_point;
    track_point.header.frame_id = track_point_frame_id_;
    track_point.header.stamp.sec = rclcpp::Time(0).seconds();
    track_point.header.stamp.nanosec = rclcpp::Time(0).nanoseconds();
    track_point.transform.translation.x = 0.0;
    track_point.transform.translation.y = 0.0;
    track_point.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped transformed_track_point;
    try
    {
        // get transformed track point (map to track point)
        // transformed_track_point = tf_buffer_->transform(track_point, map_.getFrameID(), tf2::durationFromSec(1.0));
        transformed_track_point = tf_buffer_->lookupTransform(map_.getFrameID(), track_point_frame_id_, tf2::TimePointZero);
    }
    catch(const tf2::TransformException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return false;
    }
    RCLCPP_DEBUG(get_logger(), "Move to the position x: %f, y: %f", transformed_track_point.transform.translation.x, transformed_track_point.transform.translation.y);

    grid_map::Position position(transformed_track_point.transform.translation.x, transformed_track_point.transform.translation.y);
    // move map to the position
    map_.move(position);
    return true;
}   

bool ElevationMapping::updatePrediction(const rclcpp::Time& _time_stamp)
{
    if (!use_pose_update_) return true;
    if (time_tolerance_prediction_ + _time_stamp.seconds() < map_.getTimeOfLastUpdate().seconds())
    {
        RCLCPP_ERROR(get_logger(), "Requested update with time stamp %f, but time of last update was %f.", _time_stamp.seconds(), map_.getTimeOfLastUpdate().seconds());
        return false;
    }
    else if (_time_stamp < map_.getTimeOfLastUpdate(RCL_ROS_TIME))
    {
        RCLCPP_DEBUG(get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignore update", _time_stamp.seconds(), map_.getTimeOfLastUpdate().seconds());
        return true;
    }

    // Get robot pose at requested time.
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(_time_stamp);
    if (!pose_msg)
    {
        // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
        if (pose_cache_.getOldestTime() > last_point_cloud_update_time_)
        {
            RCLCPP_ERROR(get_logger(), "The oldest pose available is at %f, requested pose at %f", pose_cache_.getOldestTime().seconds(), last_point_cloud_update_time_.seconds());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not get pose from robot for time %f", last_point_cloud_update_time_.seconds());
        }
        return false;
    }
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Map<const Eigen::MatrixXd>(pose_msg->pose.covariance.data(), 6, 6);
    Eigen::Affine3d transform;
    tf2::fromMsg(pose_msg->pose.pose, transform);
    
    // compute map variacne update from motion prediction
    robot_motion_updater_.update(map_, transform, pose_covariance, _time_stamp);

    return true;
}

bool ElevationMapping::readParameters()
{
    // elevation mapping
    use_pose_update_ = declare_parameter("use_pose_update", true);
    use_visibility_clean_up_ = declare_parameter("use_visibility_clean_up", true);
    pose_cache_size_ = declare_parameter("pose_cache_size", 10);
    track_point_frame_id_ = declare_parameter("robot_frame", "/base_link");
    time_tolerance_prediction_ = declare_parameter("time_tolerance_prediction", 0.1); // seconds
    extract_vaild_area_ = declare_parameter("extract_vaild_area", true);
    // declare_parameter("max_no_update_duration", max_no_update_duration_, 0.5);

    std::string sensor_type, sensor_frame, map_frame, robot_frame;
    sensor_frame = declare_parameter("sensor_frame", "/sensor");
    map_frame = declare_parameter("map_frame", "/map");
    robot_frame = track_point_frame_id_;
    map_.setFrameID(map_frame);
    sensor_type = declare_parameter("sensor.type", "perfect");
    if (sensor_type == "perfect") {
        sensor_processor_ = std::make_shared<PerfectSensorProcessor>(sensor_frame, map_frame, robot_frame);
    }
    else if (sensor_type == "stereo") {
        sensor_processor_ = std::make_shared<StereoSensorProcessor>(sensor_frame, map_frame, robot_frame);
    }
    else if (sensor_type == "realsense") {
        sensor_processor_ = std::make_shared<StructureLightProcessor>(sensor_frame, map_frame, robot_frame);
    }
    else {
        RCLCPP_ERROR(get_logger(), "The sensor type %s is invailed", sensor_type.c_str());
        return false;
    }

    map_.readParameters(this);
    sensor_processor_->readParameters(this);


    return true;
}

}   

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(elevation_mapping::ElevationMapping)