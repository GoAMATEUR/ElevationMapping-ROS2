#include <elevation_mapping_ros2/sesnor_processing/PerfectSensorProcessor.hpp>

int main()
{
    elevation_mapping::PerfectSensorProcessor processor("sensor", "map", "base_link");

    return 0;
}