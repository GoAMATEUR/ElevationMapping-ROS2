from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def read_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
    return params

def generate_launch_description():
    elevation_map_param_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_map.param.yaml')
    
    topic_name_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config', 
        'topic_name.yaml')
    post_processing_file = os.path.join(
        get_package_share_directory('elevation_mapping_ros2'), 
        'config', 
        'post_processing.param.yaml'
    )

    visualization_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"),
        'config',
        'visualization.yaml' 
    )
    
    params = read_yaml(elevation_map_param_file)
    params_post_processing = read_yaml(post_processing_file)
    topic_name = read_yaml(topic_name_file)
            
    # elevation_mapping_node = Node(
    #     package="elevation_mapping_ros2",
    #     name="elevation_mapping_ros2_node",
    #     executable="elevation_mapping_ros2_node",
    #     parameters=[params], 
    #     remappings=[("input/point_cloud", topic_name["input"]["point_cloud"]), 
    #                 ("input/pose", topic_name["input"]["pose_covariance"]), 
    #                 ("output/raw_map", topic_name["output"]["raw_map"])], 
    #     arguments=['--ros-args', '--log-level', 'INFO'], 
    #     output = 'screen'
    # )

    elevation_mapping_composition = Node(
        package='elevation_mapping_ros2', 
        executable='elevation_mapping_ros2_composition', 
        name='elevation_mapping_ros2_composition', 
        parameters=[params, params_post_processing], 
        remappings=[("elevation_mapping/input/point_cloud", topic_name["input"]["point_cloud"]), 
                    ("elevation_mapping/input/pose", topic_name["input"]["pose_covariance"]), 
                    ("elevation_mapping/output/raw_map", topic_name["output"]["raw_map"]), 
                    ("post_processing/output/grid_map", "filtered_map"),
                    ("post_processing/input/grid_map", topic_name["output"]["raw_map"]),
                    ],
        arguments=['--ros-args', '--log-level', 'INFO'], 
        output = 'screen'
    )
    
    elevation_raw_map_visualization = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_file]
    )
    
    return LaunchDescription([
        elevation_mapping_composition,
        elevation_raw_map_visualization, 
    ])