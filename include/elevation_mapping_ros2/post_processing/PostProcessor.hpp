#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping
{
class PostProcessor : public rclcpp::Node
{
public:
    PostProcessor(const rclcpp::NodeOptions options);
    ~PostProcessor();
private:
    bool readParameters();
    void callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _grid_map);

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_sub_map_;
    std::weak_ptr<std::remove_pointer<decltype(pub_grid_map_.get())>::type> captured_pub_grid_map_;

    filters::FilterChain<grid_map::GridMap> filter_chain_;
    grid_map::Position center_;
    grid_map::Length length_;
};

}