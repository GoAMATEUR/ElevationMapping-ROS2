#include <elevation_mapping_ros2/ElevationMapping.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<elevation_mapping::ElevationMapping>());
    rclcpp::shutdown();

    return 0;
}