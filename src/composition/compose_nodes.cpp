#include <rclcpp/rclcpp.hpp>
#include <elevation_mapping_ros2/ElevationMapping.hpp>
#include <elevation_mapping_ros2/post_processing/PostProcessor.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Start  intra-process communication
    // which can avoid serialization and deserialization, unnecessary
    // copies, and achieve lower latencies in some cases.
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true);

    // Start two nodes with a MultiThreadedExecutor
    rclcpp::Node::SharedPtr elevation_mapping = std::make_shared<elevation_mapping::ElevationMapping>(options);
    rclcpp::Node::SharedPtr post_processor = std::make_shared<elevation_mapping::PostProcessor>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(elevation_mapping);
    executor.add_node(post_processor);
    RCLCPP_INFO(elevation_mapping->get_logger(), "elevation_mapping node started!");
    RCLCPP_INFO(post_processor->get_logger(), "post_processor node started!");
    executor.spin();

    rclcpp::shutdown();

    return 0;
}