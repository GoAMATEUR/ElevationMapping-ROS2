/**
 * @brief
 * Manage all process. 
*/
#pragma once
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_msgs/msg/grid_map_info.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <elevation_mapping_ros2/ElevationMap.hpp>
#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>
#include <elevation_mapping_ros2/sesnor_processing/PerfectSensorProcessor.hpp>
#include <elevation_mapping_ros2/sesnor_processing/StereoSensorProcessor.hpp>
#include <elevation_mapping_ros2/sesnor_processing/StructureLightProcessor.hpp>
#include <elevation_mapping_ros2/TypeDef.hpp>
#include <elevation_mapping_ros2/RobotMotionUpdater.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace elevation_mapping
{
class ElevationMapping : public rclcpp::Node
{
public:
    ElevationMapping(const rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true));
    ~ElevationMapping();

private:
    // void callbackPointCloud(const sensor_msgs::msg::PointCloud2::UniquePtr _point_cloud, const geometry_msgs::msg::PoseWithCovarianceStamped _pose);
    
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_point_cloud_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> sub_pose_;
    message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped> pose_cache_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_raw_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;
    std::weak_ptr<std::remove_pointer<decltype(pub_raw_map_.get())>::type> captured_pub_raw_map_;

    // message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
    //     <sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseWithCovarianceStamped>> sync_sub_;
    
    void callbackPointcloud(const sensor_msgs::msg::PointCloud2::UniquePtr _point_cloud);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;

    // update
    bool updateMapLocation();
    bool updatePrediction(const rclcpp::Time& _time_stamp);

    // initialize
    bool readParameters();

    // tf
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // time 
    rclcpp::Time last_point_cloud_update_time_;
    rclcpp::Clock::SharedPtr clock_;

    // instances
    ElevationMap map_;
    std::shared_ptr<SensorProcessorBase> sensor_processor_;
    RobotMotionUpdater robot_motion_updater_;

    // parameters
    bool use_pose_update_ = true;
    size_t pose_cache_size_ = 10;
    bool use_visibility_clean_up_ = true;

    std::string track_point_frame_id_ = "base_link"; // /robot

    double time_tolerance_prediction_; // seconds
    bool extract_vaild_area_ = true;
    // float max_no_update_duration_;
};

}