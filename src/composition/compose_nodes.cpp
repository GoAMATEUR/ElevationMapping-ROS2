#include <rclcpp/rclcpp.hpp>
#include <elevation_mapping_ros2/ElevationMapping.hpp>
#include <elevation_mapping_ros2/post_processing/PostProcessor.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true);

    
    rclcpp::Node::SharedPtr elevation_mapping = std::make_shared<elevation_mapping::ElevationMapping>(options);
    rclcpp::Node::SharedPtr post_processor = std::make_shared<elevation_mapping::PostProcessor>(options);
    RCLCPP_INFO(elevation_mapping->get_logger(), "Elevation Mapping Node started FROM COMPOSITION.");
    RCLCPP_INFO(post_processor->get_logger(), "Post Processor Node started FROM COMPOSITION.");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(elevation_mapping);
    executor.add_node(post_processor);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}