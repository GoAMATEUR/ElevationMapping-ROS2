#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/common.h>
#include <grid_map_core/grid_map_core.hpp>

namespace elevation_mapping
{
using PointType = pcl::PointXYZRGB;
using PointCloudType = pcl::PointCloud<PointType>;
using GridMap = grid_map::GridMap;
}
