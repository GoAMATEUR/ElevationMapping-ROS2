#include <elevation_mapping_ros2/post_processing/PostProcessor.hpp>

namespace elevation_mapping
{
PostProcessor::PostProcessor(const rclcpp::NodeOptions options)
    : rclcpp::Node("post_processor", options), filter_chain_("grid_map::GridMap")
{
    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "post_processing/input/grid_map", 10, std::bind(&PostProcessor::callbackGridMap, this, std::placeholders::_1)
    );
    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "post_processing/output/grid_map", 10
    );
    pub_sub_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "post_processing/output/sub_map", 10
    );
    captured_pub_grid_map_ = pub_grid_map_;

    if (!readParameters()) 
    {
        rclcpp::shutdown();
        return;
    }
}

PostProcessor::~PostProcessor() {}

bool PostProcessor::readParameters()
{
    std::string filter_chain_parameter_name = declare_parameter("filter_chain_parameter_name", std::string("filters"));
    if (filter_chain_.configure(filter_chain_parameter_name, this->get_node_logging_interface(), this->get_node_parameters_interface()))
    {
        RCLCPP_INFO(get_logger(), "Filter chain configured");
        return true;
    }
    else 
    {
        RCLCPP_ERROR(get_logger(), "Could not configure the filter chain");
        return false;
    }
}

void PostProcessor::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _grid_map)
{
    std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();

    grid_map::GridMap input_map;
    grid_map::GridMap output_map;

    // RCLCPP_INFO(get_logger(), "subscribe map address: 0x%x", &(_grid_map->data));

    grid_map::GridMapRosConverter::fromMessage(*_grid_map, input_map);

    if (!filter_chain_.update(input_map, output_map))
    {
        RCLCPP_ERROR(get_logger(), "Could not update the grid map filter chain");
        return;
    }

    bool success;
    grid_map::GridMap sub_map = output_map.getSubmap(center_, length_, success);

    if (success)
    {
        grid_map_msgs::msg::GridMap::UniquePtr sub_map_msg = grid_map::GridMapRosConverter::toMessage(sub_map);
        // RCLCPP_INFO(get_logger(), "publish sub map address: 0x%x", &(sub_map_msg->data));
        pub_grid_map_->publish(std::move(sub_map_msg));   
    }

    grid_map_msgs::msg::GridMap::UniquePtr output_msg;
    output_msg = grid_map::GridMapRosConverter::toMessage(output_map);
    // auto pub_ptr = captured_pub_grid_map_.lock();
    // if (!pub_ptr) return;
    // RCLCPP_INFO(get_logger(), "publish map address: 0x%x", &(output_msg->data));
    
    pub_grid_map_->publish(std::move(output_msg));

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    RCLCPP_INFO(get_logger(), "post processing time: %lf", elapsed);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(elevation_mapping::PostProcessor)